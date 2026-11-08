#!/bin/bash
# Builds the full Gazebo simulator's box, "drone-simulator-gazebo-full". SIMULATOR ONLY.
#
# It starts from the Gazebo trial's box, and builds that first if it is missing (about 8
# minutes, see simulator/gazebo_trial/README.md). fly_course_in_gazebo.sh runs this for you.
set -euo pipefail

folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"

if ! docker image inspect drone-simulator-gazebo > /dev/null 2>&1; then
    echo "The Gazebo trial box 'drone-simulator-gazebo' is missing, so building it first."
    "$folder_of_this_script/../gazebo_trial/build_gazebo_image.sh"
fi

docker build --tag drone-simulator-gazebo-full "$folder_of_this_script"
