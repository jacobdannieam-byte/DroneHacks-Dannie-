#!/bin/bash
# The steps of one Gazebo trial run, inside the drone-simulator-gazebo box.
# TRIAL ONLY, SIMULATOR ONLY. Started by ../run_px4_in_gazebo.sh, which explains the steps.
#
# Arguments: the map file (relative to the repository), and "yes" or "no" for staying on
# the ground.
set -euo pipefail

map_file="$1"
stay_on_ground="$2"

trial_folder=/repository/simulator/gazebo_trial
log_folder="$trial_folder/generated/last_run"
px4_build_folder=/opt/PX4-Autopilot/build/px4_sitl_default

# These two names are set in course_world_sdf.py (WORLD_NAME) and make_gazebo_world.py
# (DRONE_NAME_IN_WORLD).
world_name=dronehacks_course
drone_name=drone

# Our own choices, in seconds: how long to wait for each program to be ready, and how long to
# let the drone settle after takeoff and landing.
longest_wait_seconds=120
settle_seconds=15

# Where Gazebo looks for "model://" names: our models first, then PX4's, which hold the x500.
export GZ_SIM_RESOURCE_PATH="$trial_folder/models:/opt/PX4-Autopilot/Tools/simulation/gz/models"

# ROS's setup scripts read variables that may not exist, which "set -u" above forbids, so we
# allow that while loading them.
set +u
source /opt/ros/humble/setup.bash
source /opt/px4_messages/install/setup.bash
set -u

# Waits until a command succeeds, checking once a second, or gives up with a message.
wait_until() {
    local description="$1"
    shift
    for _ in $(seq "$longest_wait_seconds"); do
        if "$@" > /dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    echo "Gave up waiting for $description after $longest_wait_seconds seconds. Look in simulator/gazebo_trial/generated/last_run/ for the programs' messages." >&2
    exit 1
}

make_world() {
    python3 "$trial_folder/make_gazebo_world.py" "$map_file"
    world_file="$trial_folder/generated/worlds/$(basename "${map_file%.json}").sdf"
}

start_gazebo() {
    echo "Starting Gazebo..."
    # -s: no window. -r: start running at once. --headless-rendering: draw camera pictures
    # without any screen, in software, because the box has no graphics card.
    gz sim -s -r --headless-rendering -v 2 "$world_file" > "$log_folder/gazebo.txt" 2>&1 &
    wait_until "Gazebo's clock" bash -c "gz topic -l | grep -q /world/$world_name/clock"
}

start_px4() {
    echo "Starting PX4..."
    cd "$px4_build_folder/rootfs"
    # PX4 saves its settings here. We delete them, so every run starts from the same state.
    rm -f parameters.bson parameters_backup.bson
    # PX4 writes a detailed record of the flight into a folder called "log". We point it at
    # the repository, because the box is deleted after the run.
    mkdir -p "$log_folder/px4_flight_log"
    ln -sfn "$log_folder/px4_flight_log" log
    # 4001 is PX4's number for its x500 drone in Gazebo. PX4_GZ_STANDALONE tells PX4 that
    # Gazebo is already running, and PX4_GZ_MODEL_NAME names the drone to take control of,
    # instead of adding a new one.
    PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$world_name" PX4_GZ_MODEL_NAME="$drone_name" \
        ../bin/px4 -d "$px4_build_folder/etc" -s etc/init.d-posix/rcS > "$log_folder/px4.txt" 2>&1 &
    cd /repository
    wait_until "PX4 to finish starting" grep -q "Startup script returned successfully" "$log_folder/px4.txt"
}

allow_arming_without_ground_station() {
    # NAV_DLL_ACT says what PX4 does if it loses its link to a ground station (a laptop
    # program such as QGroundControl). PX4's x500 settings make it 2, "return home", and then
    # PX4 refuses to arm until a ground station has connected. We run no ground station, so we
    # set it back to 0, "do nothing", which is PX4's own default.
    #
    # Why not set it when PX4 starts, with PX4_PARAM_NAV_DLL_ACT=0 as drone_16_settings.sh
    # does: we tried on 2026-11-07, and PX4 still refused to arm for want of a ground station.
    # PX4 applies those start-up settings first and the x500 settings after. Our explanation,
    # not yet confirmed: PX4 treats a value equal to its own default (0 here) as "not
    # changed", so the x500 settings then replaced it with 2.
    px4_command param set NAV_DLL_ACT 0
}

start_message_agent() {
    # The same agent as on the real drone. It carries PX4's messages to ROS 2.
    MicroXRCEAgent udp4 -p 8888 > "$log_folder/message_agent.txt" 2>&1 &
}

start_camera_to_ros() {
    python3 "$trial_folder/gazebo_camera_to_ros.py" > "$log_folder/camera_to_ros.txt" 2>&1 &
}

wait_for_px4_to_be_ready() {
    echo "Waiting for PX4 to be ready for takeoff..."
    # PX4 prints this once its position estimate is good enough to fly.
    wait_until "PX4 to be ready for takeoff" grep -q "Ready for takeoff" "$log_folder/px4.txt"
}

# Sends one command to the running PX4, the way PX4's own console would.
px4_command() {
    (cd "$px4_build_folder/rootfs" && "../bin/px4-$1" "${@:2}")
}

take_off() {
    echo "Taking off to 1 m with PX4's own takeoff command..."
    # Our own choice: 1 m, safely below the 1.73 m wall tops.
    px4_command param set MIS_TAKEOFF_ALT 1.0
    px4_command commander takeoff
    sleep "$settle_seconds"
}

measure() {
    local label="$1"
    echo "Measuring the camera for 20 seconds ($label)..."
    # top shows each program's processor use. 100% is one processor core fully busy.
    top -b -n 2 -d 10 -w 200 > "$log_folder/processor_load_$label.txt" &
    local top_process_id=$!
    python3 "$trial_folder/save_depth_picture.py" --label "$label" | tee "$log_folder/camera_rate_$label.txt"
    # Wait for top only. A plain "wait" would also wait for Gazebo and PX4, which never end.
    wait "$top_process_id"
    # PX4's directions: x north, y east, z DOWN, in metres. In Gazebo, PX4's "north" is the
    # course's y axis and "east" is the course's x axis, and this true position is measured
    # from the centre of the course. So on the launch mark at 1 m, x is about 0.91, y about
    # -2.51 and z about -1.0. PX4 sends this message "best effort" (it may drop one rather
    # than wait), and a listener must ask for the same, or it hears nothing.
    echo "The drone's true position (x = course y, y = course x, z = minus the height):"
    timeout 20 ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_local_position_groundtruth \
        | grep -E "^(x|y|z): " | tee "$log_folder/position_$label.txt" || true
}

land() {
    echo "Landing..."
    px4_command commander land
    sleep "$settle_seconds"
}

make_world
start_gazebo
start_px4
allow_arming_without_ground_station
start_message_agent
start_camera_to_ros
wait_for_px4_to_be_ready
if [ "$stay_on_ground" = "yes" ]; then
    measure on_ground
else
    take_off
    measure hovering
    land
fi
echo "Done."
