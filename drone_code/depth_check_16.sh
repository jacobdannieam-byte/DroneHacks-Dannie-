#!/bin/bash
# Checks drone 16's depth points against a tape measure. Run on the Mac:
#
#   drone_code/depth_check_16.sh skybeta-16.local
#
# Hold the drone level with its nose 1.0 m, then 1.5 m, then 2.0 m from a textured wall (measure
# from the camera lens), and run this at each distance. It prints whether depth_points is running,
# its last log lines (they say if OpenCV is missing), how many depth clouds come each second, and
# the median depth straight ahead, from the drone's centre and from the camera. The camera-lens
# figure should match the tape to within a few centimetres at 1 m, more at 2 m.
#
# It only reads; it sends nothing to the drone and never touches PX4.
set -e
folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=drone_16_common.sh
source "$folder_of_this_script/drone_16_common.sh"

confirm_it_is_drone_16
on_drone 'bash -s' <<'ON_DRONE'
running="$(docker exec skybeta-stack ps -eo args 2>/dev/null | grep -c 'team_nav/depth_points')"
echo "depth_points running:  $running"
echo "depth_points last log lines:"
docker exec skybeta-stack tmux capture-pane -p -J -t skybeta:competitor -S -3000 2>/dev/null \
    | grep 'team_depth_points\]' | sed 's/.*team_depth_points\]: /    /' | tail -n 3
docker exec skybeta-stack bash -c '
    for setup in /opt/ros/humble/setup.bash /root/skybeta_ws/install/setup.bash /root/team_ws/install/setup.bash; do
        [ -f $setup ] && source $setup
    done
    ros2 run team_nav depth_check --seconds 5'
ON_DRONE
