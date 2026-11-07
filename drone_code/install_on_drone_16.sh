#!/bin/bash
# Copies our team_nav package onto drone 16. Run this on the Mac.
#
# It refuses to do anything unless the computer it reaches calls itself skybeta-16. The rules say
# that logging in to any other team's drone gets us disqualified, and the drone's address
# (192.168.55.1 over USB) is the same for every drone, so the address alone proves nothing.
#
# It only adds files to ~/team_ws/src, the folder the organizers set aside for team code.
# It does not change how the drone starts. See drone_code/README.md for that step.
#
# If your computer has not connected to this drone before, you can pass your own ssh options:
#   DRONE_SSH_OPTIONS="-o UserKnownHostsFile=/path/to/file" drone_code/install_on_drone_16.sh
set -e

drone_address="${DRONE_ADDRESS:-192.168.55.1}"
drone_login="dronehacks@$drone_address"
required_drone_name="skybeta-16"
ssh_options="${DRONE_SSH_OPTIONS:-}"
folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"

# shellcheck disable=SC2086  # The options are meant to split into separate words.
actual_drone_name="$(ssh $ssh_options "$drone_login" hostname)"

if [ "$actual_drone_name" != "$required_drone_name" ]; then
    echo "Stopping. This computer calls itself '$actual_drone_name', not '$required_drone_name'."
    echo "It is not our drone, so nothing has been changed. Disconnect from it now."
    exit 1
fi

echo "Confirmed: this is $required_drone_name. Copying team_nav to the team folder."
# shellcheck disable=SC2086
ssh $ssh_options "$drone_login" "rm -rf ~/team_ws/src/team_nav && mkdir -p ~/team_ws/src"
# shellcheck disable=SC2086
scp $ssh_options -r "$folder_of_this_script/team_nav" "$drone_login:team_ws/src/team_nav"

echo "Done. The drone will build the package the next time its software starts."
