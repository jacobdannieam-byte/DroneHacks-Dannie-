#!/bin/bash
# Runs one test flight from simulator/test_flights in Gazebo, instead of in the normal
# simulator. TRIAL ONLY, SIMULATOR ONLY. Nothing here talks to a real drone.
#
# Run it on the Mac, from anywhere, the same way as simulator/run_test_flight.sh. For example,
# the whole course in our phone scan:
#
#   simulator/gazebo_trial/run_full_route_in_gazebo.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json \
#       route_test.py --drone-map drone_code/team_nav/course_maps/phone_scan_2026-11-06.json
#
# --world, if given first, chooses the course: a map file, named from the top of the repository.
# Without it, the course is the organizers' drawing. The next argument names a file in
# simulator/test_flights, and the rest go to that file.
#
# How it differs from run_test_flight.sh
# --------------------------------------
#   - It starts full_route_in_gazebo.launch.py, so Gazebo provides the physics and the course is
#     solid.
#   - Its box has its own name, so it never stops a normal simulator flight, and no network, so
#     its ROS messages can't mix with another box's.
#   - The ROS build and the flight records go to simulator/gazebo_trial/generated/route_runs/,
#     not simulator/generated_files/. The normal simulator's script shows the newest flight
#     record after each flight, and a Gazebo flight at the same time would otherwise be taken
#     for its own.
# It still shares the Mac's processors. Gazebo needs them: with all of them busy, its drone
# sank from 1 m to 0.17 m (see README.md). Don't run both simulators at once.
set -e

repository="$(cd "$(dirname "$0")/../.." && pwd)"
trial_folder="$repository/simulator/gazebo_trial"
box_name=drone-simulator-gazebo-route
runs_folder="$trial_folder/generated/route_runs"

world_map="drone_code/team_nav/course_maps/organizers_drawing.json"
if [ "$1" = "--world" ]; then
    world_map="$2"
    shift 2
fi
if [ ! -f "$repository/$world_map" ]; then
    echo "There is no map file at $world_map. Name it from the top of the repository, for example drone_code/team_nav/course_maps/phone_scan_2026-11-06.json."
    exit 1
fi
echo "Gazebo will fly in the course described by $world_map."

test_file="$1"
shift

mkdir -p "$runs_folder/ros_build" "$runs_folder/ros_install" "$runs_folder/ros_logs" "$runs_folder/flight_records"
flight_start_marker="$runs_folder/.this_flight_started"
touch "$flight_start_marker"

docker rm --force "$box_name" > /dev/null 2>&1 || true

# The extra --volume lines lay our own folders over the normal simulator's build and record
# folders, inside this box only. The normal simulator's files are not touched.
generated="/repository/simulator/generated_files"
docker run --detach --name "$box_name" \
    --network none \
    --volume "$repository:/repository" \
    --volume "$runs_folder/ros_build:$generated/ros_build" \
    --volume "$runs_folder/ros_install:$generated/ros_install" \
    --volume "$runs_folder/ros_logs:$generated/ros_logs" \
    --volume "$runs_folder/flight_records:$generated/flight_records" \
    --env "SIMULATOR_WORLD_MAP=/repository/$world_map" \
    drone-simulator-gazebo \
    bash -c "bash /repository/simulator/inside_the_box/build_our_packages.sh \
             && touch /tmp/our_packages_are_built \
             && source /repository/simulator/inside_the_box/load_ros_environment.sh \
             && ros2 launch /repository/simulator/gazebo_trial/full_route_in_gazebo.launch.py" > /dev/null

stop_the_box() {
    docker logs "$box_name" > "$runs_folder/last_simulator_log.txt" 2>&1 || true
    docker rm --force "$box_name" > /dev/null 2>&1 || true
    echo "The simulator's full output is saved in simulator/gazebo_trial/generated/route_runs/last_simulator_log.txt"

    summary_of_this_flight="$(find "$runs_folder/flight_records" -name '*_summary.json' -newer "$flight_start_marker" | sort | tail -n 1)"
    if [ -z "$summary_of_this_flight" ]; then
        echo "The referee wrote no summary for this flight, so the simulator probably stopped before the referee started. The saved output above should say why."
        return
    fi
    echo "The referee's summary of this flight ($summary_of_this_flight):"
    cat "$summary_of_this_flight"
    echo
    echo "To see the flight in 3D: python3 simulator/flight_viewer/make_flight_viewer.py ${summary_of_this_flight%_summary.json}.csv"

    # The same pass rules as simulator/run_test_flight.sh, so results compare directly. In
    # Gazebo a touch is also a real collision, which may have knocked the drone off course.
    python3 - "$summary_of_this_flight" <<'END_OF_CHECK'
import json, sys
summary = json.load(open(sys.argv[1]))
problems = []
if summary["touches"] != 0:
    problems.append(f"the drone touched something {summary['touches']} time(s)")
if summary.get("stayed_above_a_boulder_too_long"):
    problems.append("the drone stayed above a boulder too long")
if summary.get("height_estimate_went_wrong"):
    problems.append("PX4's height estimate strayed too far from the true height")
if problems:
    print("TEST FAILED: " + "; ".join(problems) + ". The summary above says where and when.")
    sys.exit(1)
END_OF_CHECK
}
trap stop_the_box EXIT

echo "Simulator starting. Waiting for the flight test to begin..."
docker exec "$box_name" bash -c "
    until [ -f /tmp/our_packages_are_built ]; do sleep 1; done
    source /repository/simulator/inside_the_box/load_ros_environment.sh
    python3 /repository/simulator/test_flights/$test_file $*
"
