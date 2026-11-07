#!/bin/bash
# Looks at drone 16's computer and the organizers' software on it. READ ONLY.
#
# check_drone_16.sh sends this file to the drone over SSH and runs it there. It changes nothing:
# every command below only reads or prints. It never prints passwords: from the drone's settings
# file it reads only the lines that say which program starts.
set -u

section() {
    echo
    echo "=== $1"
}

section "Which drone is this?"
hostname

section "The organizers' health check (host/doctor.sh). Their README says running it is always fine."
# Their check prints colour codes, which we remove so the saved report is readable.
(cd ~/skybeta && timeout 120 host/doctor.sh 2>&1 | sed 's/\x1b\[[0-9;]*m//g')

section "Is the organizers' software running? It should show skybeta-stack."
docker ps --format '{{.Names}}   {{.Status}}'

section "Which program does the drone start? The organizers' demo is competitor_example."
# The demo climbs to 1.5 m and flies a 5 m square, which would hit the course walls, as soon as
# someone arms the drone and flips the switch. It must be replaced before the propellers go on.
grep -E '^COMPETITOR_(PKG|LAUNCH)=' /etc/skybeta/drone.env

section "Programs running inside the organizers' software. square_mission is the demo."
docker exec skybeta-stack pgrep -af 'square_mission|flight_interface|visual_slam|micro.?xrce|oak' \
    | cut -c1-160

section "ROS network number. It must differ from every other drone at the venue."
# ROS programs on the same Wi-Fi with the same number hear each other's messages.
docker exec skybeta-stack printenv ROS_DOMAIN_ID

section "Is the flight controller talking to this computer? (the organizers' check_topics.sh)"
docker exec skybeta-stack bash -lc 'check_topics.sh' 2>&1 | tail -20

section "Free disk space, for flight recordings"
df -h ~ | tail -1

section "Temperatures and processor load, one sample"
timeout 3 tegrastats --interval 1000 2>/dev/null | head -1
