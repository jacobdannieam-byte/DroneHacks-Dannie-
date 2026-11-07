#!/bin/bash
# Checks that every part of drone 16 is alive, WITHOUT FLYING. Run this on the Mac, with the
# drone's computer (the Jetson) connected to the Mac by USB.
#
# READ ONLY. It looks and listens; it never arms the drone, never spins a motor, never sends a
# target and never changes a file or setting on the drone.
#
# What it does, in order:
#   1. Makes sure the computer on the end of the USB cable is drone 16. Every drone answers on the
#      same USB address, and logging in to another team's drone gets us disqualified.
#   2. Runs on_the_drone/look_at_the_drone.sh on the drone: the organizers' own health check, and
#      whether their software and their demo program are running.
#   3. Runs on_the_drone/listen_to_every_sensor.py inside the organizers' software on the drone:
#      it listens to every sensor for a few seconds and says which ones are sending data.
#
# Everything it prints is also saved in results/, named after the date and time.
#
# Usage, from the top of the repository:
#   drone_code/hardware_checks/check_drone_16.sh
set -u

drone_address="${DRONE_ADDRESS:-192.168.55.1}"
drone_login="dronehacks@$drone_address"
required_drone_name="skybeta-16"

# Read from drone 16 over USB on 2026-11-06. The user confirmed it was the Jetson they had plugged
# in, and it calls itself skybeta-16. If drone 16's computer is ever reinstalled, this changes.
drone_16_key_fingerprint="SHA256:/w3BVlUTQFzMiJ5qlRAjy2O8aN6ohE4aq4QILYZxTZk"

folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"
results_folder="$folder_of_this_script/results"
mkdir -p "$results_folder"
results_file="$results_folder/check_$(date +%Y-%m-%d_%H-%M-%S).txt"

# Keep a copy of everything printed from here on.
exec > >(tee "$results_file") 2>&1

echo "Drone 16 hardware check, $(date). READ ONLY: nothing will move."

# --- 1. Is this drone 16? ---------------------------------------------------------------------
# We keep drone 16's identity key in a file of our own, rather than the Mac's usual list, because
# the Mac has an older entry for a different Jetson at the same address.
known_hosts_file="$(mktemp)"
trap 'rm -f "$known_hosts_file"' EXIT
ssh-keyscan -T 5 -t ed25519 "$drone_address" > "$known_hosts_file" 2>/dev/null
actual_fingerprint="$(ssh-keygen -lf "$known_hosts_file" 2>/dev/null | awk '{print $2}')"
if [ -z "$actual_fingerprint" ]; then
    echo "STOPPED: no drone answered at $drone_address. Check that the USB cable is plugged into"
    echo "the Jetson's USB-C port and that the drone is powered, then try again."
    exit 1
fi
if [ "$actual_fingerprint" != "$drone_16_key_fingerprint" ]; then
    echo "STOPPED: the computer at $drone_address is not the drone 16 we know (its identity key"
    echo "differs). It may be another team's drone. Unplug it. Nothing was changed."
    exit 1
fi

# -F /dev/null ignores the Mac's ssh settings, which send the name "jetson" to a different
# computer. BatchMode means it never stops to ask for a password.
ssh_to_drone=(ssh -F /dev/null -o "UserKnownHostsFile=$known_hosts_file" -o StrictHostKeyChecking=yes
              -o BatchMode=yes -o ConnectTimeout=8 "$drone_login")

actual_drone_name="$("${ssh_to_drone[@]}" hostname 2>/dev/null)"
if [ -z "$actual_drone_name" ]; then
    echo "STOPPED: drone 16 answered, but the Mac couldn't log in without a password. The Mac's"
    echo "login key may not be on the drone. Run this once and type the drone password yourself:"
    echo "  ssh-copy-id -o UserKnownHostsFile=$known_hosts_file dronehacks@$drone_address"
    exit 1
fi
if [ "$actual_drone_name" != "$required_drone_name" ]; then
    echo "STOPPED: this computer calls itself '$actual_drone_name', not '$required_drone_name'."
    echo "It is not our drone. Unplug it now. Nothing was changed."
    exit 1
fi
echo "Confirmed: this is $required_drone_name."

# --- 2. The drone's computer and the organizers' software --------------------------------------
"${ssh_to_drone[@]}" 'bash -s' < "$folder_of_this_script/on_the_drone/look_at_the_drone.sh"

# --- 3. Every sensor ---------------------------------------------------------------------------
echo
echo "=== Listening to every sensor (this takes about 15 seconds)"
# The Python file travels to the drone through the SSH connection and into the organizers'
# container, where ROS and the drone's message types are installed. Nothing is copied to disk.
"${ssh_to_drone[@]}" \
    'docker exec -i skybeta-stack bash -c "source skybeta_env.sh && python3 -"' \
    < "$folder_of_this_script/on_the_drone/listen_to_every_sensor.py"
sensor_result=$?

echo
echo "Saved this report to $results_file"
exit $sensor_result
