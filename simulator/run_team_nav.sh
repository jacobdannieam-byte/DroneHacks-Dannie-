#!/bin/bash
# Flies our real mission program (drone_code/team_nav) once in the simulator, and saves the record.
# SIMULATOR ONLY. Run this on the Mac, from anywhere:
#
#   simulator/run_team_nav.sh            # the course stages (mission:=stages)
#   simulator/run_team_nav.sh route      # the route in drone_code/team_nav/missions/route_waypoints.json
#   simulator/run_team_nav.sh route route_fly_through_degrees:=45
#   simulator/run_team_nav.sh route route_file:=route_waypoints_scan.json
#   simulator/run_team_nav.sh route route_file:=route_waypoints_drawing.json drift_correction:=true
#   SIMULATOR_ARGUMENTS="tracking_drift_metres_per_metre:=0.0" simulator/run_team_nav.sh route
#
# Any words after the mission are passed to our launch file as launch arguments (name:=value).
# SIMULATOR_ARGUMENTS, if set, is passed to the simulator's own launch file. Neither changes a default.
#
# What it does, in order:
#   1. Starts the whole simulator, except the organizers' flight interface. Our own launch file
#      starts that program, and the organizers' guide warns that two flight interfaces fight over
#      the control flags.
#   2. Launches drone_code/team_nav/launch/team_nav.launch.py, the same file the drone uses, with
#      three settings for the simulator:
#        target_system:=1     the simulator's PX4 is system 1 (drone 16's is 16, the default)
#        vision_check:=false  the simulator has no camera images, so nothing reports camera
#                             tracking health; this makes the mission treat it as healthy. The
#                             mission accepts false only from this script, inside the simulator
#                             box (see TEAM_NAV_STARTED_BY in drone_code/team_nav/team_nav/mission_node.py).
#        depth_topic:=/simulator_only/depth_points   Kenny's pretend depth camera, which the drift
#                             correction reads. It is only listened to with drift_correction:=true.
#   3. Runs test_flights/team_nav_flight.py. It presses start through the pretend remote control
#      (arm, then flip the switch to Position), watches the flight, and prints a RESULT line.
#   4. Saves the records in simulator/generated_files/flight_records/ and prints the referee's summary.
#
# Expected today with the stages: the hand-over, a climb to 0.8 m, a hold (there is no line input
# yet), the landing with the push-down, and the motors switching off. With the route: the same start,
# every waypoint in turn, then the landing; the watcher reports whether it reached the last waypoint.
#
# Each run gets a fresh simulator, because the pretend remote control starts a run only once.
set -e

mission_kind="${1:-stages}"
shift || true
team_nav_arguments="$*"
# The watcher must check the same route the mission flies.
route_file=route_waypoints.json
for argument in "$@"; do
    case "$argument" in
        route_file:=*) route_file="${argument#route_file:=}" ;;
    esac
done
simulator_arguments="${SIMULATOR_ARGUMENTS:-}"
if [ "$mission_kind" != "stages" ] && [ "$mission_kind" != "route" ]; then
    echo "The mission must be 'stages' or 'route', not '$mission_kind'. For example: simulator/run_team_nav.sh route"
    exit 1
fi

repository="$(cd "$(dirname "$0")/.." && pwd)"
box_name=drone-simulator
generated_files="$repository/simulator/generated_files"
flight_records_folder="$generated_files/flight_records"
team_nav_log="$generated_files/last_team_nav_log.txt"
mkdir -p "$flight_records_folder"
rm -f "$team_nav_log"

# This file marks when this flight began, so that we never show a summary from an earlier flight.
flight_start_marker="$generated_files/.this_flight_started"
touch "$flight_start_marker"

# Remove a box left behind if an earlier run was interrupted.
docker rm --force "$box_name" > /dev/null 2>&1 || true

docker run --detach --name "$box_name" \
    --volume "$repository:/repository" \
    drone-simulator \
    bash -c "bash /repository/simulator/inside_the_box/build_our_packages.sh \
             && touch /tmp/our_packages_are_built \
             && source /repository/simulator/inside_the_box/load_ros_environment.sh \
             && ros2 launch course_simulator whole_simulator.launch.py start_flight_interface:=false $simulator_arguments" > /dev/null

# Always save the records and stop the box when this script ends, even if the flight fails.
save_the_records_and_stop() {
    sleep 2   # let the last lines of our mission's log reach the file
    docker logs "$box_name" > "$generated_files/last_simulator_log.txt" 2>&1 || true
    docker rm --force "$box_name" > /dev/null 2>&1 || true
    echo "The simulator's full output is saved in simulator/generated_files/last_simulator_log.txt"

    # The file names contain the date and time, so sorting them puts the newest last.
    summary_of_this_flight="$(find "$flight_records_folder" -name '*_summary.json' -newer "$flight_start_marker" | sort | tail -n 1)"
    if [ -z "$summary_of_this_flight" ]; then
        echo "The referee wrote no summary for this flight, so the simulator probably stopped before the referee started. The saved output above should say why."
        return
    fi
    record_name="${summary_of_this_flight%_summary.json}"
    if [ -f "$team_nav_log" ]; then
        cp "$team_nav_log" "${record_name}_team_nav_log.txt"
        echo "Our mission's log is saved as simulator/generated_files/flight_records/$(basename "$record_name")_team_nav_log.txt"
    fi
    echo "The flight record is simulator/generated_files/flight_records/$(basename "$record_name").csv"
    echo "The referee's summary of this flight:"
    cat "$summary_of_this_flight"
    echo
}
trap save_the_records_and_stop EXIT

echo "Simulator starting (extra simulator arguments: ${simulator_arguments:-none}). Our $mission_kind mission"
echo "starts once our packages are built (extra launch arguments: ${team_nav_arguments:-none})..."

# Our launch file runs in the background inside the box. Its output is our mission's log.
# TEAM_NAV_STARTED_BY is what lets the mission accept vision_check:=false. PYTHONUNBUFFERED makes
# every log line reach the file straight away.
docker exec --detach \
    --env TEAM_NAV_STARTED_BY=simulator/run_team_nav.sh \
    --env PYTHONUNBUFFERED=1 \
    "$box_name" bash -c "
        until [ -f /tmp/our_packages_are_built ]; do sleep 1; done
        source /repository/simulator/inside_the_box/load_ros_environment.sh
        ros2 launch team_nav team_nav.launch.py target_system:=1 vision_check:=false \
            depth_topic:=/simulator_only/depth_points mission:=$mission_kind $team_nav_arguments \
            > /repository/simulator/generated_files/last_team_nav_log.txt 2>&1
    "

docker exec "$box_name" bash -c "
    until [ -f /tmp/our_packages_are_built ]; do sleep 1; done
    source /repository/simulator/inside_the_box/load_ros_environment.sh
    python3 /repository/simulator/test_flights/team_nav_flight.py --mission $mission_kind --route-file $route_file
"
