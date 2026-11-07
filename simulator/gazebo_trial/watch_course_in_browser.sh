#!/bin/bash
# Shows a course in Gazebo, live and in 3D, in your web browser. TRIAL ONLY, SIMULATOR ONLY.
#
# Run it on the Mac, from anywhere, then open https://app.gazebosim.org/visualization and
# connect to ws://localhost:9002. Turn the view with the mouse. Press Ctrl+C here to stop.
#
#   simulator/gazebo_trial/watch_course_in_browser.sh            (our phone scan)
#   simulator/gazebo_trial/watch_course_in_browser.sh <map file> (any other map)
#
# Only Gazebo runs here: no PX4 and no ROS. So the box can safely have one port open to the
# Mac (9002, reachable only from this Mac), without hearing any other simulator's messages.
# Your browser does the 3D drawing, so the box needs no graphics card.
set -euo pipefail

repository="$(cd "$(dirname "$0")/../.." && pwd)"
map_file="${1:-drone_code/team_nav/course_maps/phone_scan_2026-11-06.json}"
box_name=drone-simulator-gazebo-browser-view
trial=/repository/simulator/gazebo_trial

docker rm --force "$box_name" > /dev/null 2>&1 || true
trap 'docker rm --force "$box_name" > /dev/null 2>&1 || true' EXIT

echo "Starting Gazebo with $map_file."
echo "Then open https://app.gazebosim.org/visualization and connect to ws://localhost:9002"
docker run --rm --name "$box_name" \
    --publish 127.0.0.1:9002:9002 \
    --volume "$repository:/repository" --workdir /repository \
    --env GZ_SIM_RESOURCE_PATH="$trial/models:/opt/PX4-Autopilot/Tools/simulation/gz/models" \
    drone-simulator-gazebo bash -c "
        python3 $trial/make_gazebo_world.py $map_file
        world=$trial/generated/worlds/\$(basename ${map_file%.json}).sdf
        gz sim -s -r -v 1 \"\$world\" &
        gz launch $trial/inside_the_box/browser_view.gzlaunch
    "
