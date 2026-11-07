#!/bin/bash
# Builds the Gazebo trial box, "drone-simulator-gazebo". TRIAL ONLY.
#
# It starts from the normal simulator box, "drone-simulator", and leaves that box
# unchanged. Build the normal box first with simulator/build_simulator_image.sh.
#
# Measured on 2026-11-07: took about 8 minutes on an M1 Mac, and adds about 2.5 GB of disk
# ("docker images" lists drone-simulator at 5.5 GB and drone-simulator-gazebo at 8.0 GB).
set -euo pipefail

folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"

if ! docker image inspect drone-simulator > /dev/null 2>&1; then
    echo "The normal simulator box 'drone-simulator' does not exist yet." >&2
    echo "Build it first with simulator/build_simulator_image.sh, then run this again." >&2
    exit 1
fi

docker build --tag drone-simulator-gazebo "$folder_of_this_script"
