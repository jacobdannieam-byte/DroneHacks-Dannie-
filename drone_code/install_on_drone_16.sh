#!/bin/bash
# Copies our team_nav package onto drone 16. Run this on the Mac, from anywhere:
#
#   drone_code/install_on_drone_16.sh skybeta-16.local     # over the DroneHacks Wi-Fi
#   drone_code/install_on_drone_16.sh                      # over USB (192.168.55.1)
#
# The address can also come from DRONE_HOST. The script refuses to do anything unless the computer
# it reaches calls itself skybeta-16 (see drone_16_common.sh).
#
# It only replaces ~/team_ws/src/team_nav, in the folder the organizers set aside for team code.
# The drone builds the whole of ~/team_ws each time its software (the "stack") starts, and if any
# package fails to build, nothing of ours starts. So restart and check afterwards:
#   drone_code/restart_drone_16.sh skybeta-16.local
set -e
folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=drone_16_common.sh
source "$folder_of_this_script/drone_16_common.sh"

confirm_it_is_drone_16
echo "Copying team_nav to ~/team_ws/src/team_nav on the drone."
on_drone "rm -rf ~/team_ws/src/team_nav && mkdir -p ~/team_ws/src"
# Copy without Python's cache folders, which would only be rebuilt there.
staging="$(mktemp -d)"
trap 'rm -rf "$staging"' EXIT
rsync -a --exclude '__pycache__' --exclude '.pytest_cache' "$folder_of_this_script/team_nav" "$staging/"
copy_to_drone -r "$staging/team_nav" "$drone_login:team_ws/src/team_nav"

echo "Done. It is built the next time the stack starts. To restart it now and check the drone:"
echo "  drone_code/restart_drone_16.sh $drone_host"
