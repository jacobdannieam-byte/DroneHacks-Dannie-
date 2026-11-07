#!/bin/bash
# Copies our flight records from drone 16 to this Mac. Run this on the Mac, from anywhere, with
# the drone reachable as skybeta-16.local (venue Wi-Fi or the cable), or its address in DRONE_ADDRESS.
#
#   drone_code/collect_flight_records.sh
#   drone_code/collect_flight_records.sh --from-inside-container
#
# READ ONLY on the drone: it never deletes, moves or changes anything there. It only lists and
# copies. Running it again copies only what is new, and never deletes anything on the Mac either.
#
# It refuses to do anything unless the computer it reaches calls itself skybeta-16. The rules say
# that logging in to any other team's drone gets us disqualified, and the drone's address
# (192.168.55.1 over USB) is the same for every drone, so the address alone proves nothing.
#
# The records end up in flight_records_from_drone/skybeta-16/ at the top of the repository. Git
# ignores that folder, so records are never committed by accident.
#
# Where the records are on the drone
# ----------------------------------
# Our programs write into ~/team_ws/flight_records inside the organizers' container
# (drone_code/team_nav/team_nav/run_log.py). The container borrows team_ws from the drone's own
# disk, so normally the same files are in /home/dronehacks/team_ws/flight_records outside the
# container, and this script copies them from there. If the recorder warned that its folder is
# inside the container's own filesystem (records_space.py), use --from-inside-container, which
# streams them out of the running container instead. Do that BEFORE the container is restarted,
# because restarting it deletes them.
#
# The password: ssh asks for it itself, at most once per run of this script, because the first
# connection is kept open for the others. Type it yourself; it is never stored anywhere.
#
# If your computer has not connected to this drone before, you can pass your own ssh options,
# as for install_on_drone_16.sh:
#   DRONE_SSH_OPTIONS="-o UserKnownHostsFile=/path/to/file" drone_code/collect_flight_records.sh
set -e
set -o pipefail  # A failure anywhere in a pipe counts as a failure.

drone_address="${DRONE_ADDRESS:-skybeta-16.local}"
drone_login="dronehacks@$drone_address"
required_drone_name="skybeta-16"
ssh_options="${DRONE_SSH_OPTIONS:-}"
# From drone_code/team_nav/team_nav/run_log.py: the records folder, relative to the home folder.
records_folder_in_home="team_ws/flight_records"
# From the organizers' setup on drone 16 (drone_code/hardware_checks/results/).
container_name="skybeta-stack"

repository="$(cd "$(dirname "$0")/.." && pwd)"
local_folder="$repository/flight_records_from_drone/$required_drone_name"

from_inside_container=false
if [ "$1" = "--from-inside-container" ]; then
    from_inside_container=true
elif [ -n "$1" ]; then
    echo "Unknown option '$1'. The only option is --from-inside-container."
    exit 1
fi

# One ssh connection is opened and reused, so the password is asked for only once. It is a
# file on this Mac that ssh deletes when the connection closes; it holds no password.
connection_folder="$(mktemp -d)"
shared_connection=(-o ControlMaster=auto -o "ControlPath=$connection_folder/connection" -o ControlPersist=120)
close_connection() {
    ssh "${shared_connection[@]}" -O exit "$drone_login" > /dev/null 2>&1 || true
    rm -rf "$connection_folder"
}
trap close_connection EXIT

on_drone() {
    # shellcheck disable=SC2086  # The options are meant to split into separate words.
    ssh "${shared_connection[@]}" $ssh_options "$drone_login" "$@"
}

# --- 1. Is this drone 16? -------------------------------------------------------------------------
actual_drone_name="$(on_drone hostname)"
if [ "$actual_drone_name" != "$required_drone_name" ]; then
    echo "Stopping. This computer calls itself '$actual_drone_name', not '$required_drone_name'."
    echo "It is not our drone, so nothing has been copied. Disconnect from it now."
    exit 1
fi
echo "Confirmed: this is $required_drone_name."

# --- 2. Say which folders the container borrows (for information only) ---------------------------
echo "The organizers' container borrows these folders from the drone's disk (drone -> container):"
on_drone "docker inspect -f '{{range .Mounts}}  {{.Source}} -> {{.Destination}}{{println}}{{end}}' $container_name" \
    || echo "  (could not ask Docker; this does not stop the copy)"

mkdir -p "$local_folder"

# --- 3a. Copy out of the running container ---------------------------------------------------------
if [ "$from_inside_container" = true ]; then
    container_copy="$local_folder/from_inside_container"
    mkdir -p "$container_copy"
    echo "Copying ~/$records_folder_in_home from inside the container to $container_copy ..."
    # tar packs the folder into one stream on the drone and unpacks it here; nothing is written
    # on the drone. \$HOME is the container's home folder, expanded inside the container.
    on_drone "docker exec $container_name sh -c 'tar -C \"\$HOME/team_ws\" -cf - flight_records'" \
        | tar -xf - -C "$container_copy"
    echo "Done. The records are in $container_copy"
    exit 0
fi

# --- 3b. Copy from the drone's own disk -------------------------------------------------------------
if ! on_drone "test -d ~/$records_folder_in_home"; then
    echo "There is no ~/$records_folder_in_home on the drone, outside the container."
    echo "Either nothing has been recorded yet, or the records are inside the container only."
    echo "Look at the folder list above, then try: drone_code/collect_flight_records.sh --from-inside-container"
    exit 1
fi

echo "Copying ~/$records_folder_in_home to $local_folder ..."
if on_drone "command -v rsync" > /dev/null; then
    # rsync copies only files that are new or have changed. Without --delete it never deletes.
    rsync -a -v -e "ssh ${shared_connection[*]} $ssh_options" \
        "$drone_login:$records_folder_in_home/" "$local_folder/"
else
    # shellcheck disable=SC2086
    scp "${shared_connection[@]}" $ssh_options -r "$drone_login:$records_folder_in_home/." "$local_folder/"
fi
echo "Done. The records are in $local_folder"
echo "PX4's own flight logs are not copied by this script; see drone_code/COLLECTING_RECORDS.md."
