#!/bin/bash
# Runs one test flight on a freshly started simulator, and then stops the simulator.
# Run this on the Mac, from anywhere. For example:
#
#   simulator/run_test_flight.sh landing_test.py --with-push-down
#   simulator/run_test_flight.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json landing_test.py
#
# Two options may come first, in either order:
#   --world <map file> chooses the course the simulator flies in: a course map file, named from
#       the top of the repository. Without it, the simulator uses the organizers' drawing.
#   --try-px4-setting NAME=VALUE runs PX4 with one setting that drone 16 does not have, to see
#       what changing it would do. Drone 16 itself is never changed by this.
# The next argument names a file in simulator/test_flights. The rest go to that file.
# Each flight gets a fresh simulator, because a simulated drone starts only once.
set -e

repository="$(cd "$(dirname "$0")/.." && pwd)"
box_name=drone-simulator

world_setting=()
px4_setting_to_try=()
while [ "$1" = "--world" ] || [ "$1" = "--try-px4-setting" ]; do
    if [ "$1" = "--world" ]; then
        if [ ! -f "$repository/$2" ]; then
            echo "There is no map file at $2. Name it from the top of the repository, for example drone_code/team_nav/course_maps/phone_scan_2026-11-06.json."
            exit 1
        fi
        # The simulator's programs read this setting (see course_simulator/world.py).
        world_setting=(--env "SIMULATOR_WORLD_MAP=/repository/$2")
        echo "The simulator will fly in the course described by $2."
    else
        if [[ "$2" != *=* ]]; then
            echo "--try-px4-setting needs NAME=VALUE, for example EKF2_RNG_K_GATE=3."
            exit 1
        fi
        # simulator/px4_changes/start_px4.sh reads this and applies it after drone 16's settings.
        px4_setting_to_try=(--env "PX4_SETTING_TO_TRY=$2")
        echo "THIS FLIGHT TRIES A PX4 SETTING THAT DRONE 16 DOES NOT HAVE: $2"
    fi
    shift 2
done

test_file="$1"
shift

flight_records_folder="$repository/simulator/generated_files/flight_records"
mkdir -p "$flight_records_folder"

# This file marks when this flight began, so that we never show a summary from an earlier flight.
flight_start_marker="$repository/simulator/generated_files/.this_flight_started"
touch "$flight_start_marker"

# Remove a box left behind if an earlier run was interrupted.
docker rm --force "$box_name" > /dev/null 2>&1 || true

# --network none: the box gets no network at all. Its programs talk to each other inside the box,
# so they don't need one. Without this, ROS programs in any other box on this Mac (such as the
# Gazebo trial) hear this box's drone and this box hears theirs, which mixes up two drones.
docker run --detach --name "$box_name" \
    --network none \
    --volume "$repository:/repository" \
    "${world_setting[@]}" \
    "${px4_setting_to_try[@]}" \
    drone-simulator \
    bash -c "bash /repository/simulator/inside_the_box/build_our_packages.sh \
             && touch /tmp/our_packages_are_built \
             && source /repository/simulator/inside_the_box/load_ros_environment.sh \
             && ros2 launch course_simulator whole_simulator.launch.py" > /dev/null

# Always stop the box when this script ends, even if the test flight fails.
stop_the_box() {
    docker logs "$box_name" > "$repository/simulator/generated_files/last_simulator_log.txt" 2>&1 || true
    docker rm --force "$box_name" > /dev/null 2>&1 || true
    echo "The simulator's full output is saved in simulator/generated_files/last_simulator_log.txt"

    # The file names contain the date and time, so sorting them puts the newest last.
    summary_of_this_flight="$(find "$flight_records_folder" -name '*_summary.json' -newer "$flight_start_marker" | sort | tail -n 1)"
    if [ -z "$summary_of_this_flight" ]; then
        echo "The referee wrote no summary for this flight, so the simulator probably stopped before the referee started. The saved output above should say why."
        return
    fi
    echo "The referee's summary of this flight:"
    cat "$summary_of_this_flight"
    echo

    # The drone must never touch anything. The pretend drone flies straight through walls, so
    # the flight itself may look fine; the referee's count of touches is what shows the problem.
    touch_count="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["touches"])' "$summary_of_this_flight")"
    if [ "$touch_count" != "0" ]; then
        echo "TEST FAILED: the drone touched something $touch_count time(s). The referee's messages in last_simulator_log.txt say where."
        exit 1
    fi

    # A boulder under the bottom laser can upset PX4's height estimate (boulder_watch.py says how).
    if [ "$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["stayed_above_a_boulder_too_long"])' "$summary_of_this_flight")" = "True" ]; then
        echo "TEST FAILED: the drone stayed above a boulder too long. The summary's times_above_boulders says which one."
        exit 1
    fi
    if [ "$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["height_estimate_went_wrong"])' "$summary_of_this_flight")" = "True" ]; then
        echo "TEST FAILED: PX4's height estimate strayed too far from the true height. The summary says how far, and when."
        exit 1
    fi
}
trap stop_the_box EXIT

echo "Simulator starting. Waiting for the flight test to begin..."
docker exec "$box_name" bash -c "
    # Our packages may still be compiling. The box creates this file when they are done.
    until [ -f /tmp/our_packages_are_built ]; do sleep 1; done
    source /repository/simulator/inside_the_box/load_ros_environment.sh
    python3 /repository/simulator/test_flights/$test_file $*
"
