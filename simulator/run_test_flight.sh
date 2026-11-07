#!/bin/bash
# Runs one test flight on a freshly started simulator, and then stops the simulator.
# Run this on the Mac, from anywhere. For example:
#
#   simulator/run_test_flight.sh landing_test.py --with-push-down
#
# The first argument names a file in simulator/test_flights. The rest go to that file.
# Each flight gets a fresh simulator, because a simulated drone starts only once.
set -e

test_file="$1"
shift
repository="$(cd "$(dirname "$0")/.." && pwd)"
box_name=drone-simulator

flight_records_folder="$repository/simulator/generated_files/flight_records"
mkdir -p "$flight_records_folder"

# This file marks when this flight began, so that we never show a summary from an earlier flight.
flight_start_marker="$repository/simulator/generated_files/.this_flight_started"
touch "$flight_start_marker"

# Remove a box left behind if an earlier run was interrupted.
docker rm --force "$box_name" > /dev/null 2>&1 || true

docker run --detach --name "$box_name" \
    --volume "$repository:/repository" \
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
