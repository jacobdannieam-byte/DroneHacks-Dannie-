#!/bin/bash
# Shared by the drone 16 scripts in this folder (install, check, restart, depth check). Run on the
# Mac; it is read with "source", never run by itself.
#
# The drone's address comes from the script's first argument, else DRONE_HOST, else 192.168.55.1
# (the drone over USB). Over the DroneHacks Wi-Fi we use skybeta-16.local.
#
# Every script first checks that the computer it reaches calls itself skybeta-16. The rules say
# that logging in to any other team's drone gets us disqualified, and 192.168.55.1 is the same
# address for every drone over USB, so the address alone proves nothing.
#
# One ssh connection is opened and reused for 60 s, so a password is asked for once, not at every
# step. Extra ssh options can be given, for example a known-hosts file:
#   DRONE_SSH_OPTIONS="-o UserKnownHostsFile=/path/to/file" drone_code/check_drone_16.sh

REQUIRED_DRONE_NAME="skybeta-16"
DRONE_LOGIN_NAME="dronehacks"

drone_host="${1:-${DRONE_HOST:-192.168.55.1}}"
drone_login="$DRONE_LOGIN_NAME@$drone_host"
# shellcheck disable=SC2206  # The options are meant to split into separate words.
ssh_options=(-o ConnectTimeout=10 -o ControlMaster=auto -o "ControlPath=$HOME/.ssh/drone16-%C"
             -o ControlPersist=60 ${DRONE_SSH_OPTIONS:-})

on_drone() {
    ssh "${ssh_options[@]}" "$drone_login" "$@"
}

copy_to_drone() {
    scp "${ssh_options[@]}" "$@"
}

confirm_it_is_drone_16() {
    local actual_drone_name
    if ! actual_drone_name="$(on_drone hostname)"; then
        echo "Could not log in to $drone_login. Check the address (USB: 192.168.55.1, Wi-Fi: skybeta-16.local)"
        echo "and that this computer is on the same network as the drone."
        exit 1
    fi
    if [ "$actual_drone_name" != "$REQUIRED_DRONE_NAME" ]; then
        echo "Stopping. This computer calls itself '$actual_drone_name', not '$REQUIRED_DRONE_NAME'."
        echo "It is not our drone, so nothing has been done. Disconnect from it now."
        exit 1
    fi
    echo "Confirmed: $drone_host is $REQUIRED_DRONE_NAME."
}
