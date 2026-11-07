#!/bin/bash
# Checks that drone 16 is ready to fly our code, and says why if it is not. Run this on the Mac:
#
#   drone_code/check_drone_16.sh skybeta-16.local     # over the DroneHacks Wi-Fi
#   drone_code/check_drone_16.sh                      # over USB (192.168.55.1)
#
# It prints one plain line each for: the skybeta version (takeover needs 0.3.3 or later), our
# COMPETITOR_PKG and COMPETITOR_LAUNCH settings, which programs are running (team_nav, exactly one
# flight_interface, no square_mission), whether the last team_ws build succeeded, the link to PX4,
# camera tracking's position and whether it is inside the start limits, camera tracking's rate and
# longest gap over 5 s, the left camera's FAST corner count, and the mission's last 3 log lines.
# Then READY TO FLY, or NOT READY with the reasons. It takes about 20 s.
#
# It only reads. The checks run on the drone (drone_16_check_on_drone.sh), after confirming that the
# drone is really skybeta-16 (drone_16_common.sh). The start limits can be changed with
# ORIGIN_CHECK_HORIZONTAL_METRES and ORIGIN_CHECK_VERTICAL_METRES, to match the launch arguments.
set -e
folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=drone_16_common.sh
source "$folder_of_this_script/drone_16_common.sh"

confirm_it_is_drone_16
on_drone "bash -s -- ${ORIGIN_CHECK_HORIZONTAL_METRES:-1.0} ${ORIGIN_CHECK_VERTICAL_METRES:-0.3}" \
    < "$folder_of_this_script/drone_16_check_on_drone.sh"
