#!/bin/bash
# Builds the Linux box the simulator runs in. Run this once on the Mac. It takes
# about 20 minutes the first time. See simulator/Dockerfile for what goes inside.
#
# You need Docker running first. On our Mac that is Colima, started with:
#   colima start --cpu 6 --memory 10 --disk 45 --vm-type vz \
#       --mount-type virtiofs --mount "$(pwd)":w
set -e

folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"

docker build --tag drone-simulator "$folder_of_this_script"
