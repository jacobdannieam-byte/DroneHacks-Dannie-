#!/bin/bash
# Runs one test flight on a freshly started simulator, and then stops the simulator.
# Run this on the Mac, from anywhere. For example:
#
#   simulator/run_test_flight.sh landing_test.py --with-push-down
#   simulator/run_test_flight.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json landing_test.py
#
# --world, if given first, chooses the course the simulator flies in: a course map file, named
# from the top of the repository. Without it, the simulator uses the organizers' drawing.
# The next argument names a file in simulator/test_flights. The rest go to that file.
# Each flight gets a fresh simulator, because a simulated drone starts only once.
set -e

repository="$(cd "$(dirname "$0")/.." && pwd)"
box_name=drone-simulator

world_setting=()
if [ "$1" = "--world" ]; then
    if [ ! -f "$repository/$2" ]; then
        echo "There is no map file at $2. Name it from the top of the repository, for example drone_code/team_nav/course_maps/phone_scan_2026-11-06.json."
        exit 1
    fi
    # The simulator's programs read this setting (see course_simulator/world.py).
    world_setting=(--env "SIMULATOR_WORLD_MAP=/repository/$2")
    echo "The simulator will fly in the course described by $2."
    shift 2
fi

test_file="$1"
shift

flight_records_folder="$repository/simulator/generated_files/flight_records"
mkdir -p "$flight_records_folder"

# This file marks when this flight began, so that we never show a summary from an earlier flight.
flight_start_marker="$repository/simulator/generated_files/.this_flight_started"
touch "$flight_start_marker"

# Remove a box left behind if an earlier run was interrupted.
docker rm --force "$box_name" > /dev/null 2>&1 || true

docker run --detach --name "$box_name" \
    --volume "$repository:/repository" \
    "${world_setting[@]}" \
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
}
trap stop_the_box EXIT

echo "Simulator starting. Waiting for the flight test to begin..."
docker exec "$box_name" bash -c "
    # Our packages may still be compiling. The box creates this file when they are done.
    until [ -f /tmp/our_packages_are_built ]; do sleep 1; done
    source /repository/simulator/inside_the_box/load_ros_environment.sh
    python3 /repository/simulator/test_flights/$test_file $*
"
