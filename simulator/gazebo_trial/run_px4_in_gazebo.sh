#!/bin/bash
# Flies PX4's x500 drone, with our pretend depth camera, in a Gazebo copy of the course, and
# measures the camera. TRIAL ONLY, SIMULATOR ONLY. Nothing here talks to a real drone.
#
# Run it on the Mac, from anywhere:
#
#   simulator/gazebo_trial/run_px4_in_gazebo.sh                    (the organizers' drawing)
#   simulator/gazebo_trial/run_px4_in_gazebo.sh drone_code/team_nav/course_maps/phone_scan_2026-11-06.json
#   simulator/gazebo_trial/run_px4_in_gazebo.sh --stay-on-ground   (no takeoff, only the camera)
#
# What happens, inside the drone-simulator-gazebo box:
#   1. make_gazebo_world.py turns the map into a Gazebo world, with the drone on the launch mark.
#   2. Gazebo starts with that world, with no window.
#   3. PX4 starts and takes control of the drone in Gazebo. It flies Gazebo's drone instead of
#      using its own built-in physics.
#   4. The message agent starts, so PX4's messages appear on ROS 2, as on the real drone.
#   5. gazebo_camera_to_ros.py starts, so the depth pictures appear on ROS 2 too.
#   6. Unless --stay-on-ground is given: once PX4 says it is ready, PX4's own takeoff command
#      lifts the drone to 1 m and holds it there. This uses PX4's built-in takeoff, not our
#      mission code, and GPS-style position, which Gazebo provides and the real drone lacks.
#   7. save_depth_picture.py listens to the camera on ROS 2 for 20 seconds, prints its rate,
#      and saves a picture. The processor load of each program is saved too.
#   8. The drone lands, and the box is stopped.
#
# Everything the programs printed is saved in simulator/gazebo_trial/generated/last_run/.
set -euo pipefail

repository="$(cd "$(dirname "$0")/../.." && pwd)"
map_file="drone_code/team_nav/course_maps/organizers_drawing.json"
stay_on_ground=no
for argument in "$@"; do
    case "$argument" in
        --stay-on-ground) stay_on_ground=yes ;;
        *) map_file="$argument" ;;
    esac
done

box_name=drone-simulator-gazebo-trial
log_folder="$repository/simulator/gazebo_trial/generated/last_run"
mkdir -p "$log_folder"

# Remove a box left behind if an earlier run was interrupted.
docker rm --force "$box_name" > /dev/null 2>&1 || true

# Always save the logs and stop the box when this script ends, even after a failure.
stop_the_box() {
    docker rm --force "$box_name" > /dev/null 2>&1 || true
    echo "Everything the programs printed is in simulator/gazebo_trial/generated/last_run/"
}
trap stop_the_box EXIT

# --network none: the box gets no network, only its own internal one, which is all Gazebo, PX4,
# the message agent and ROS need to talk to each other. Without it, ROS programs in two boxes
# on the same Mac hear each other. On 2026-11-07 this happened: our normal simulator's referee
# heard this box's drone, and this box's PX4 could hear the other simulator's pretend remote
# control, pretend laser and commands.
docker run --detach --name "$box_name" \
    --network none \
    --volume "$repository:/repository" \
    --workdir /repository \
    drone-simulator-gazebo sleep infinity > /dev/null

docker exec "$box_name" bash /repository/simulator/gazebo_trial/inside_the_box/run_trial.sh \
    "$map_file" "$stay_on_ground"
