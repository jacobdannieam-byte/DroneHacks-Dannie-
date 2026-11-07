#!/bin/bash
# Copies our test-flight package (team_smoke_flights) onto drone 16 and checks that it builds there.
# Run this on the Mac, with the drone's computer connected by USB.
#
# It only ADDS the folder ~/team_ws/src/team_smoke_flights. It doesn't touch any other team
# package on the drone, and it doesn't change which program the drone starts. README.md in this
# folder explains that separate step.
#
# It refuses to do anything unless the computer it reaches is drone 16, because every drone has
# the same USB address and logging in to another team's drone gets us disqualified.
set -eu

drone_address="${DRONE_ADDRESS:-192.168.55.1}"
drone_login="dronehacks@$drone_address"
required_drone_name="skybeta-16"
# Read from drone 16 over USB on 2026-11-06 (see drone_code/hardware_checks/check_drone_16.sh).
drone_16_key_fingerprint="SHA256:/w3BVlUTQFzMiJ5qlRAjy2O8aN6ohE4aq4QILYZxTZk"
folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"

known_hosts_file="$(mktemp)"
trap 'rm -f "$known_hosts_file"' EXIT
ssh-keyscan -T 5 -t ed25519 "$drone_address" > "$known_hosts_file" 2>/dev/null
if [ "$(ssh-keygen -lf "$known_hosts_file" 2>/dev/null | awk '{print $2}')" != "$drone_16_key_fingerprint" ]; then
    echo "Stopping: the computer at $drone_address is not the drone 16 we know, or nothing answered."
    exit 1
fi
ssh_options=(-F /dev/null -o "UserKnownHostsFile=$known_hosts_file" -o StrictHostKeyChecking=yes
             -o BatchMode=yes -o ConnectTimeout=8)
if [ "$(ssh "${ssh_options[@]}" "$drone_login" hostname)" != "$required_drone_name" ]; then
    echo "Stopping: this computer is not $required_drone_name. Nothing was changed."
    exit 1
fi
echo "Confirmed: this is $required_drone_name."

echo "Copying team_smoke_flights to ~/team_ws/src/team_smoke_flights on the drone."
ssh "${ssh_options[@]}" "$drone_login" "rm -rf ~/team_ws/src/team_smoke_flights && mkdir -p ~/team_ws/src"
scp -q "${ssh_options[@]}" -r "$folder_of_this_script" "$drone_login:team_ws/src/team_smoke_flights"

echo "Building only this package inside the organizers' software, to check it builds on the drone."
ssh "${ssh_options[@]}" "$drone_login" \
    'docker exec skybeta-stack bash -c "source skybeta_env.sh && cd /root/team_ws && colcon build --packages-select team_smoke_flights 2>&1 | tail -5"'
echo "Done. The drone still starts the same program as before; README.md says how to switch."
