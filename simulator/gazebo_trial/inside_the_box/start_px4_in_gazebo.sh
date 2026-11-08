#!/bin/bash
# Starts Gazebo and PX4 for the full-route flight, in place of the normal simulator's
# simulator/px4_changes/start_px4.sh. TRIAL ONLY, SIMULATOR ONLY. Nothing here talks to a drone.
#
# full_route_in_gazebo.launch.py runs this as its first program. It:
#   1. makes the drone 16 model (make_drone_16_model.py) and the Gazebo world from the map in
#      SIMULATOR_WORLD_MAP, with Gazebo's origin where the drone rests on the launch mark
#      (make_gazebo_world.py explains why);
#   2. starts Gazebo, with no window;
#   3. starts PX4 flying the Gazebo drone, with drone 16's settings;
#   4. undoes the few setting changes made by PX4's x500 start-up file (see below);
#   5. starts the camera bridge, so Gazebo's depth pictures appear on ROS 2.
# It then waits, and stops Gazebo and PX4 when it is stopped itself.
#
# Which settings PX4 gets
# -----------------------
# All of simulator/px4_changes/drone_16_settings.sh except SIH_*, which are for PX4's own
# physics, not running here. The Gazebo drone has drone 16's motor positions, weight and thrust,
# so drone 16's motor positions (CA_ROTOR*) and hover power (MPC_THR_HOVER) fit it. It works out
# its position like drone 16: pretend camera tracking and pretend bottom laser, no GPS.
#
# Then the settings the drone model needs (generated/models/drone_16_like/px4_settings.txt), and
# PX4's "x500 in Gazebo" start-up file (airframe 4001), which knows how to talk to Gazebo.
set -euo pipefail

trial_folder=/repository/simulator/gazebo_trial
log_folder="$trial_folder/generated/last_route_run"
px4_build_folder=/opt/PX4-Autopilot/build/px4_sitl_default
mkdir -p "$log_folder"

# These names are set in course_world_sdf.py (WORLD_NAME) and make_gazebo_world.py
# (DRONE_NAME_IN_WORLD).
world_name=dronehacks_course
drone_name=drone

# Our own choice, in seconds: how long to wait for each program to be ready.
longest_wait_seconds=120

settings_left_out_pattern='^PX4_PARAM_SIH_'
model_settings_file="$trial_folder/generated/models/drone_16_like/px4_settings.txt"

# Where Gazebo looks for "model://" names: the generated drone 16 model, our other models, and
# PX4's models, which the drone 16 model borrows its looks and sensors from.
export GZ_SIM_RESOURCE_PATH="$trial_folder/generated/models:$trial_folder/models:/opt/PX4-Autopilot/Tools/simulation/gz/models"

wait_until() {
    local description="$1"
    shift
    for _ in $(seq "$longest_wait_seconds"); do
        if "$@" > /dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    echo "Gave up waiting for $description after $longest_wait_seconds seconds. Look in simulator/gazebo_trial/generated/last_route_run/." >&2
    exit 1
}

# Sends one command to the running PX4, the way PX4's own console would.
px4_command() {
    (cd "$px4_build_folder/rootfs" && "../bin/px4-$1" "${@:2}")
}

# Prints every setting PX4 gets, as "NAME VALUE" lines: drone 16's, without the ones listed
# above, then the drone model's.
px4_settings() {
    (
        set +u
        # start_px4 below exports every setting, so forget those first, or the model's settings
        # would be listed twice.
        for exported_name in $(compgen -e | grep '^PX4_PARAM_'); do
            unset "$exported_name"
        done
        source /repository/simulator/px4_changes/drone_16_settings.sh
        env | grep '^PX4_PARAM_' | grep -Ev "$settings_left_out_pattern" | sed -e 's/^PX4_PARAM_//' -e 's/=/ /'
    )
    cat "$model_settings_file"
    if [ "${GAZEBO_ISOLATED_DDS:-0}" = 1 ]; then
        printf 'UXRCE_DDS_DOM_ID %s\nUXRCE_DDS_AG_IP 2130706433\nUXRCE_DDS_PTCFG 1\n' "$ROS_DOMAIN_ID"
    fi
}

make_drone_model_and_world() {
    python3 "$trial_folder/make_drone_16_model.py"
    world_file="$(cd /repository && python3 "$trial_folder/make_gazebo_world.py" "$SIMULATOR_WORLD_MAP" --for-full-simulator | sed 's/^Wrote //')"
    world_file="/repository/$world_file"
    echo "Gazebo world: $world_file"
}

start_gazebo() {
    echo "Starting Gazebo..."
    local render_args=(--headless-rendering)
    case "${GAZEBO_RENDER_BACKEND:-egl}" in
        egl) ;;
        wslg_d3d12)
            glxinfo -B > "$log_folder/renderer.txt" 2>&1
            grep -q 'Accelerated: yes' "$log_folder/renderer.txt"
            grep -q 'OpenGL renderer string: D3D12 (NVIDIA' "$log_folder/renderer.txt"
            render_args=()
            ;;
        *) echo 'Unsupported Gazebo rendering backend' >&2; return 1 ;;
    esac
    gz sim -s -r "${render_args[@]}" -v 2 "$world_file" > "$log_folder/gazebo.txt" 2>&1 &
    gazebo_process_id=$!
    wait_until "Gazebo's clock" bash -c "gz topic -l | grep -q /world/$world_name/clock"
}

start_px4() {
    echo "Starting PX4 with drone 16's settings..."
    cd "$px4_build_folder/rootfs"
    rm -f parameters.bson parameters_backup.bson
    mkdir -p "$log_folder/px4_flight_log"
    ln -sfn "$log_folder/px4_flight_log" log
    # The settings as start-up settings (PX4 reads every PX4_PARAM_ variable), the same way the
    # normal simulator gives them (simulator/px4_changes/start_px4.sh).
    while read -r name value; do
        export "PX4_PARAM_$name=$value"
    done < <(px4_settings)
    # 4001 is PX4's number for its x500 in Gazebo. PX4_GZ_STANDALONE says Gazebo is already
    # running, and PX4_GZ_MODEL_NAME names the drone to take control of.
    PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$world_name" PX4_GZ_MODEL_NAME="$drone_name" \
        ../bin/px4 -d "$px4_build_folder/etc" -s etc/init.d-posix/rcS > "$log_folder/px4.txt" 2>&1 &
    px4_process_id=$!
    cd /repository
    wait_until "PX4 to finish starting" grep -q "Startup script returned successfully" "$log_folder/px4.txt"
}

undo_the_x500_start_up_file() {
    # PX4 reads the start-up settings first and the drone type's own start-up file after. That
    # file can replace a start-up setting that equals PX4's built-in value (seen on 2026-11-07
    # with NAV_DLL_ACT). The normal simulator has the same order, so for every setting except
    # the ones below, PX4 ends up with the same value here as there, which is what we want.
    #
    # The exceptions are what the x500's file (airframe 4001) changes and the normal
    # simulator's file (airframe 10040) does not. We compared the two files on 2026-11-07:
    #   - MPC_THR_HOVER: the x500's file makes it 0.60; drone 16's 0.5 is PX4's built-in value.
    #   - NAV_DLL_ACT: the x500's file makes it 2, which makes PX4 refuse to arm until a ground
    #     station connects. We run none. 0, "do nothing", is PX4's built-in value.
    # And the drone model's own settings, so that nothing else can replace them.
    echo "Undoing the x500 start-up file's changes, now that PX4 is running..."
    px4_command param set MPC_THR_HOVER 0.5 > /dev/null
    px4_command param set NAV_DLL_ACT 0 > /dev/null
    while read -r name value; do
        px4_command param set "$name" "$value" > /dev/null
    done < "$model_settings_file"
}

record_the_settings_px4_uses() {
    # For the record: the value PX4 really uses for every setting we gave it, as "NAME VALUE".
    # "param show -q" prints only the value, so we print the name in front of it.
    while read -r name _; do
        echo "$name $(px4_command param show -q "$name" 2>&1)"
    done < <(px4_settings) > "$log_folder/px4_settings_in_use.txt" || true
    echo "The settings PX4 uses are listed in simulator/gazebo_trial/generated/last_route_run/px4_settings_in_use.txt"
}

start_camera_bridge() {
    python3 "$trial_folder/gazebo_camera_to_ros.py" > "$log_folder/camera_to_ros.txt" 2>&1 &
    camera_process_id=$!
}

stop_everything() {
    kill "${px4_process_id:-}" "${gazebo_process_id:-}" "${camera_process_id:-}" 2> /dev/null || true
}
trap stop_everything EXIT

make_drone_model_and_world
start_gazebo
start_px4
undo_the_x500_start_up_file
record_the_settings_px4_uses
if [ "${HYBRID_GAZEBO_RUN:-0}" != 1 ]; then
    start_camera_bridge
fi
echo "Gazebo and PX4 are running."
wait "$px4_process_id"
