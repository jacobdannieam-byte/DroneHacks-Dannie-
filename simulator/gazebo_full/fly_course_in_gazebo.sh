#!/bin/bash
# Flies the team's real course program (ros2 run team_nav course_run) in the full Gazebo
# simulator, and saves a picture of the path. SIMULATOR ONLY. Nothing here talks to a drone.
#
# Run it on the Mac, from anywhere. For example:
#
#   simulator/gazebo_full/fly_course_in_gazebo.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json
#   simulator/gazebo_full/fly_course_in_gazebo.sh --world <world map> --run-mode competition -- --map organizers_drawing.json
#
# Options, which come first, in any order:
#   --world <map file>       What is really there: the course Gazebo builds, the referee judges
#                            and the picture shows. Named from the top of the repository.
#                            Without it, real_course_2026-11-08.json.
#   --run-mode <mode>        practice (the default) or competition, given to course_run as
#                            TEAM_NAV_RUN_MODE, as on the drone.
#   --longest-run-seconds N  Stop course_run (as with Ctrl+C, so it lands) after N seconds.
#                            Default 360.
#   --stop-after-stage <name>  Given to the flight program as TEAM_NAV_STOP_AFTER_STAGE, as
#                            drone.sh does on the drone. Practice mode only. Default: none.
# Everything else, or everything after "--", goes to course_run unchanged. The drone's own map
# is course_run's --map; without it, course_run uses its default map, whatever --world says.
#
# Each run gets its own folder in simulator/generated_files/gazebo_full_runs/, with the path
# picture, a short summary, the referee's records, course_run's records, the stereo check and
# every log. README.md lists them.
#
# The box has no network (--network none), so its ROS messages can't mix with another box's, or
# reach a drone. It shares the Mac's processors: don't run the normal simulator at the same time.
set -euo pipefail

repository="$(cd "$(dirname "$0")/../.." && pwd)"
full_folder="$repository/simulator/gazebo_full"
box_name=drone-simulator-gazebo-full-run
image_name=drone-simulator-gazebo-full

world_map="drone_code/team_nav/course_maps/real_course_2026-11-08.json"
run_mode=practice
# Our own choice: about 6 minutes, longer than any whole-course run so far (under 4 minutes).
longest_run_seconds=360
stop_after_stage=""
course_run_arguments=()
while [ $# -gt 0 ]; do
    case "$1" in
        --world) world_map="$2"; shift 2 ;;
        --run-mode) run_mode="$2"; shift 2 ;;
        --longest-run-seconds) longest_run_seconds="$2"; shift 2 ;;
        --stop-after-stage) stop_after_stage="$2"; shift 2 ;;
        --) shift; course_run_arguments+=("$@"); break ;;
        *) course_run_arguments+=("$1"); shift ;;
    esac
done
if [ ! -f "$repository/$world_map" ]; then
    echo "There is no map file at $world_map. Name it from the top of the repository, for example drone_code/team_nav/course_maps/phone_scan_2026-11-06.json."
    exit 1
fi

if ! docker image inspect "$image_name" > /dev/null 2>&1; then
    "$full_folder/build_gazebo_full_image.sh"
fi

run_name="$(date +%Y%m%d_%H%M%S)"
run_folder="$repository/simulator/generated_files/gazebo_full_runs/$run_name"
run_folder_in_box="/repository/simulator/generated_files/gazebo_full_runs/$run_name"
build_folder="$repository/simulator/generated_files/gazebo_full_build"
mkdir -p "$run_folder/referee" "$run_folder/course_run_records" "$build_folder/ros_build" "$build_folder/ros_install" "$build_folder/ros_logs"
echo "World: $world_map. Run mode: $run_mode. Stop after stage: ${stop_after_stage:-(none)}. Extra course_run arguments: ${course_run_arguments[*]:-(none)}."
echo "This run's folder: simulator/generated_files/gazebo_full_runs/$run_name"

docker rm --force "$box_name" > /dev/null 2>&1 || true

# The extra --volume lines lay our own folders over the normal simulator's build folders and
# the referee's record folder, inside this box only, so the normal simulator's files are
# untouched and the referee writes straight into this run's folder. TEAM_NAV_RUN_MODE and
# TEAM_NAV_RECORDS_FOLDER are set for the whole box, so the team's depth program, started by the
# launch file, writes its records next to course_run's.
generated="/repository/simulator/generated_files"
docker run --detach --name "$box_name" \
    --network none \
    --volume "$repository:/repository" \
    --volume "$build_folder/ros_build:$generated/ros_build" \
    --volume "$build_folder/ros_install:$generated/ros_install" \
    --volume "$build_folder/ros_logs:$generated/ros_logs" \
    --volume "$run_folder/referee:$generated/flight_records" \
    --env "SIMULATOR_WORLD_MAP=/repository/$world_map" \
    --env "GAZEBO_FULL_RUN_FOLDER=$run_folder_in_box" \
    --env "TEAM_NAV_RUN_MODE=$run_mode" \
    --env "TEAM_NAV_RECORDS_FOLDER=$run_folder_in_box/course_run_records" \
    "$image_name" \
    bash -c "bash /repository/simulator/inside_the_box/build_our_packages.sh \
             && touch /tmp/our_packages_are_built \
             && source /repository/simulator/inside_the_box/load_ros_environment.sh \
             && ros2 launch /repository/simulator/gazebo_full/whole_stack_in_gazebo.launch.py" > /dev/null

stop_the_box_and_draw() {
    docker logs "$box_name" > "$run_folder/simulator_log.txt" 2>&1 || true
    docker rm --force "$box_name" > /dev/null 2>&1 || true
    echo "Drawing the path and writing the summary..."
    docker run --rm --network none --volume "$repository:/repository" "$image_name" \
        python3 /repository/simulator/gazebo_full/draw_run.py "$run_folder_in_box" "/repository/$world_map" || true
    echo
    cat "$run_folder/summary.txt" 2>/dev/null || echo "No summary was written. Look in $run_folder/simulator_log.txt."
    echo "Everything from this run is in simulator/generated_files/gazebo_full_runs/$run_name"
}
trap stop_the_box_and_draw EXIT

# printf %q quotes each argument so that it reaches course_run unchanged inside the box.
quoted_course_run_arguments=""
if [ ${#course_run_arguments[@]} -gt 0 ]; then
    quoted_course_run_arguments="$(printf ' %q' "${course_run_arguments[@]}")"
fi

echo "Simulator starting. This takes about a minute before the drone moves..."
docker exec "$box_name" bash -c "
    until [ -f /tmp/our_packages_are_built ]; do sleep 1; done
    source /repository/simulator/inside_the_box/load_ros_environment.sh
    export TEAM_NAV_RUN_MODE=$run_mode
    export TEAM_NAV_STOP_AFTER_STAGE=$(printf '%q' "$stop_after_stage")
    export TEAM_NAV_RECORDS_FOLDER=$run_folder_in_box/course_run_records
    bash /repository/simulator/gazebo_full/inside_the_box/check_cameras_then_fly.sh $longest_run_seconds $quoted_course_run_arguments
" 2>&1 | tee "$run_folder/flight_program_output.txt"
