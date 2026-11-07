#!/bin/bash
# Copies our code from this Mac to drone 16 and builds it there. Run it on the Mac, from the top
# of the repository:
#
#     drone_code/install_on_drone_16.sh
#
# The rule: change code only on the Mac, in this repository. Then run this script. Do not edit
# code on the drone. Then the repository and the drone always agree.
#
# What it does
# ------------
#   1. It makes sure that the computer is skybeta-16. If it is not, it stops. A login to the
#      drone of a different team causes disqualification. An address alone proves nothing,
#      because over USB each drone has the address 192.168.55.1.
#   2. It makes sure that ~/team_ws/src/team_nav is ours or empty. If it finds a different
#      team_nav there, it stops and changes nothing.
#   3. It copies our two packages into ~/team_ws/src:
#        drone_code/team_nav        ->  ~/team_ws/src/team_nav
#        drone_code/smoke_flights   ->  ~/team_ws/src/team_smoke_flights
#   4. It builds them with the organizers' build script, team_build.sh.
#   5. It puts a link to our helper, drone.sh, in ~/team_ws.
#
# It does not restart the drone's software. Programs that run now keep the old code. To use the
# new code, type "./drone.sh start watch_only" on the drone.
#
# It never arms the drone, never turns a motor and never changes a PX4 setting or a settings file.
#
# The drone's address: the default is skybeta-16.local. This name operates on the venue's
# "DroneHacks Wifi" and over the USB cable. To use the cable address directly:
#     DRONE_ADDRESS=192.168.55.1 drone_code/install_on_drone_16.sh
set -e

drone_address="${DRONE_ADDRESS:-skybeta-16.local}"
drone_login="dronehacks@$drone_address"
required_drone_name="skybeta-16"
ssh_options="${DRONE_SSH_OPTIONS:-}"
folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"

# A file that only our team_nav has. Step 2 looks for it.
FILE_THAT_ONLY_OUR_TEAM_NAV_HAS="team_nav/course_run.py"

# shellcheck disable=SC2086  # The options must split into separate words.
actual_drone_name="$(ssh $ssh_options "$drone_login" hostname)"
if [ "$actual_drone_name" != "$required_drone_name" ]; then
    echo "Stopping. This computer is '$actual_drone_name', not '$required_drone_name'."
    echo "It is not our drone. Nothing was changed. Disconnect from it now."
    exit 1
fi
echo "Confirmed: this is $required_drone_name."

# shellcheck disable=SC2086
ssh $ssh_options "$drone_login" "
    set -e
    if [ -d ~/team_ws/src/team_nav ] && [ ! -f ~/team_ws/src/team_nav/$FILE_THAT_ONLY_OUR_TEAM_NAV_HAS ]; then
        echo 'Stopping. ~/team_ws/src/team_nav is not our team_nav. Nothing was changed.'
        echo 'Move it to ~/archive first (drone_code/README.md tells how).'
        exit 1
    fi
    mkdir -p ~/team_ws/src ~/team_ws/flight_records
    touch ~/team_ws/flight_records/COLCON_IGNORE
"
# The COLCON_IGNORE file tells the build to skip the records folder, which can hold many files.

echo "Copying our two packages into ~/team_ws/src."
# --delete removes files that we deleted on the Mac, but only in these two folders.
# shellcheck disable=SC2086
rsync -a --delete --exclude __pycache__ --exclude .pytest_cache -e "ssh $ssh_options" \
    "$folder_of_this_script/team_nav/" "$drone_login:team_ws/src/team_nav/"
# shellcheck disable=SC2086
rsync -a --delete --exclude __pycache__ --exclude .pytest_cache -e "ssh $ssh_options" \
    "$folder_of_this_script/smoke_flights/" "$drone_login:team_ws/src/team_smoke_flights/"

echo "Building them with the organizers' build script (about a minute)."
# shellcheck disable=SC2086
ssh $ssh_options "$drone_login" "docker exec skybeta-stack team_build.sh"

# shellcheck disable=SC2086
ssh $ssh_options "$drone_login" "chmod +x ~/team_ws/src/team_nav/drone.sh && ln -sfn src/team_nav/drone.sh ~/team_ws/drone.sh"

echo "Done. The new code is on the drone. To use it, type on the drone:"
echo "    cd ~/team_ws && ./drone.sh start watch_only"
