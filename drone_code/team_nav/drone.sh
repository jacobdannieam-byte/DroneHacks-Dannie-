#!/bin/bash
# The one command that runs our programs on drone 16. REAL DRONE. Type it on the Jetson (the
# drone's computer), after you log in with ssh:
#
#     cd ~/team_ws
#     ./drone.sh status                         show what runs and if the hand-over is on
#     ./drone.sh start watch_only               start a launch file (here: nothing can fly)
#     ./drone.sh hand_test all_table_checks     run one of our programs, with its arguments
#
# drone_code/TEST_SEQUENCE.md gives the commands for each test, in order.
#
# How the drone's software is arranged
# ------------------------------------
# The organizers' software runs in a Docker container that has the name skybeta-stack. In the
# container, a terminal program ("tmux") keeps one window for each part: camera tracking, the
# link to PX4, and a window with the name "competitor". When the software starts, the competitor
# window builds ~/team_ws/src and starts our launch/team_nav.launch.py. That file starts
# watch_only, so no program can fly after a restart.
#
# "start" stops the program in the competitor window and starts the launch file that you name,
# in a new window with the name "team_launch". It does not change a settings file and needs no password.
#
# Only one launch file can run at a time. Each launch file starts the organizers'
# flight_interface, and only one program must send commands to PX4. Thus "start" waits until
# the old flight_interface has stopped.
#
# This script never arms the drone, never turns a motor and never changes a PX4 setting.
# "start" does not operate while the drone is armed.
set -u

CONTAINER="skybeta-stack"
TMUX_SESSION="skybeta"
ORGANIZERS_WINDOW="competitor"
OUR_WINDOW="team_launch"

# In the container, the drone's ~/team_ws is /root/team_ws. The organizers' skybeta_env.sh
# makes the packages built from ~/team_ws/src available.
RECORDS_FOLDER="/root/team_ws/flight_records"
SET_UP_ROS="source skybeta_env.sh"

# How a running flight_interface shows up in the process list.
FLIGHT_INTERFACE_PROCESS="skybeta_flight_interface/flight_interface"

# Our own choices: how long to wait for a launch file to stop, and for a new one to come up.
SECONDS_TO_WAIT_FOR_STOP=20
SECONDS_TO_WAIT_FOR_START=40

# Settings our programs read from the environment (run_mode.py, run_log.py). If you set one
# before ./drone.sh, it is passed into the container.
PASSED_SETTINGS="TEAM_NAV_RUN_MODE TEAM_NAV_STOP_AFTER_STAGE TEAM_NAV_RECORDS_FOLDER"


# ---- Small helpers -------------------------------------------------------------------------------

in_container() {
    docker exec "$CONTAINER" bash -c "$1"
}

window_exists() {
    in_container "tmux list-windows -t $TMUX_SESSION -F '#{window_name}'" | grep -qx "$1"
}

flight_interface_is_running() {
    in_container "pgrep -f $FLIGHT_INTERFACE_PROCESS" > /dev/null 2>&1
}

give_records_back_to_dronehacks() {
    # The container runs as root, so files it writes belong to root. Hand them back, so the
    # dronehacks login can copy and delete them.
    in_container "chown -R \$HOST_UID:\$HOST_GID $RECORDS_FOLDER 2>/dev/null; true"
}

# Prints the organizers' latest summary of the drone: armed, landed, PX4 connected, hand-over.
drone_summary() {
    in_container "source skybeta_env.sh && timeout 5 ros2 topic echo --once --field data /skybeta/state 2>/dev/null" |
        python3 -c '
import json, sys
text = sys.stdin.read().strip().strip("-").strip().strip("\x27")
if not text:
    print("No summary from flight_interface in 5 s: it is not running, or ROS is not reachable.")
    sys.exit(0)
state = json.loads(text)
print("PX4 connected:", state.get("px4_link"), "  armed:", state.get("armed"),
      "  landed:", state.get("landed"), "  mode:", state.get("nav_state_name"),
      "  our program has control:", state.get("offboard_active"))
'
}

drone_is_armed() {
    drone_summary | grep -q "armed: True"
}


# ---- status --------------------------------------------------------------------------------------

show_status() {
    echo "== Windows in the drone's software:"
    in_container "tmux list-windows -t $TMUX_SESSION -F '  #{window_name}'"
    echo "== Launch files running now:"
    in_container "ps -eo args | grep '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch' | sed 's|.*ros2 launch|  ros2 launch|'"
    echo "== Hand-over setting (auto_offboard_on_arm; True means arming and flipping to Position hands control to a program):"
    in_container "source skybeta_env.sh && timeout 10 ros2 param get /flight_interface auto_offboard_on_arm 2>&1 | sed 's/^/  /'"
    echo "== The drone now:"
    drone_summary | sed 's/^/  /'
    echo "== Newest record folders ($RECORDS_FOLDER):"
    in_container "ls -t $RECORDS_FOLDER 2>/dev/null | grep -v COLCON_IGNORE | head -5 | sed 's/^/  /'"
}


# ---- start -------------------------------------------------------------------------------------

stop_window() {
    local window_name="$1"
    if ! window_exists "$window_name"; then
        return 0
    fi
    echo "Stopping the program in the '$window_name' window (Ctrl+C)."
    in_container "tmux send-keys -t $TMUX_SESSION:$window_name C-c"
    for second in $(seq "$SECONDS_TO_WAIT_FOR_STOP"); do
        if ! flight_interface_is_running; then break; fi
        sleep 1
    done
    if flight_interface_is_running; then
        echo "Stopping. flight_interface is still running $SECONDS_TO_WAIT_FOR_STOP s after Ctrl+C, so a second one"
        echo "must not be started. Attach and look: docker exec -it $CONTAINER tmux attach -t $TMUX_SESSION"
        exit 1
    fi
    in_container "tmux kill-window -t $TMUX_SESSION:$window_name 2>/dev/null; true"
}

start_launch_file() {
    local launch_name="${1:-}"
    local package_and_file
    case "$launch_name" in
        watch_only)
            package_and_file="team_nav watch_only.launch.py" ;;
        test_flights)
            package_and_file="team_nav test_flights.launch.py" ;;
        rise_and_land)
            package_and_file="team_smoke_flights rise_and_land.launch.py" ;;
        course_run)
            # Its hand-over stays off until someone sets HAND_OVER_ON in course_run.launch.py.
            package_and_file="team_nav course_run.launch.py" ;;
        *)
            echo "Choose what to start: watch_only, test_flights, rise_and_land or course_run."
            exit 1 ;;
    esac

    if drone_is_armed; then
        echo "Stopping. The drone is ARMED. Disarm it before changing what runs. Nothing was changed."
        exit 1
    fi

    stop_window "$ORGANIZERS_WINDOW"
    stop_window "$OUR_WINDOW"
    if flight_interface_is_running; then
        echo "Stopping. A flight_interface still runs in a window that this script does not know."
        echo "A second one must not start. Restart the drone software: docker rm -f $CONTAINER"
        exit 1
    fi

    echo "Starting: ros2 launch $package_and_file (in the '$OUR_WINDOW' window)."
    # When the launch ends, the window keeps a shell open so its last messages stay readable.
    in_container "tmux new-window -d -t $TMUX_SESSION -n $OUR_WINDOW \"bash -c '$SET_UP_ROS && ros2 launch $package_and_file; echo The launch file has ended.; exec bash'\""

    for second in $(seq "$SECONDS_TO_WAIT_FOR_START"); do
        if flight_interface_is_running; then break; fi
        sleep 1
    done
    sleep 3  # Give flight_interface a moment to set up its settings before reading them.
    show_status
}


# ---- Running one of our programs -----------------------------------------------------------------

run_our_program() {
    local settings=()
    for name in $PASSED_SETTINGS; do
        if [ -n "${!name:-}" ]; then
            settings+=(-e "$name=${!name}")
        fi
    done
    # -it gives the program this terminal, so it can ask you to press Enter and Ctrl+C reaches it.
    docker exec -it "${settings[@]}" "$CONTAINER" bash -c "$SET_UP_ROS && ros2 run team_nav \"\$@\"" drone "$@"
    local exit_code=$?
    give_records_back_to_dronehacks
    return $exit_code
}


# ---- Choosing what to do -------------------------------------------------------------------------

if [ "$(hostname)" != "skybeta-16" ]; then
    echo "Stopping. This computer is $(hostname), not skybeta-16. Run this on drone 16's Jetson."
    exit 1
fi

case "${1:-}" in
    status)
        show_status ;;
    start)
        start_launch_file "${2:-}" ;;
    hand_test|practice_flight|course_run|flight_recorder)
        run_our_program "$@" ;;
    *)
        sed -n '2,10p' "$0"
        exit 1 ;;
esac
