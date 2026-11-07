#!/bin/bash
# Compiles our ROS packages. It runs inside the box, each time the simulator starts. It is
# quick after the first time, because the results are kept in simulator/generated_files.
#
# It builds the simulator's packages, the organizers' code, and the code in drone_code. The
# code in drone_code is what also runs on the real drone, so the simulator tests the same code.
set -e

source /opt/ros/humble/setup.bash
source /opt/px4_messages/install/setup.bash

cd /repository/simulator/generated_files
# colcon only accepts --log-base before the word "build".
colcon --log-base ros_logs build --symlink-install \
    --base-paths ../ros_packages ../organizers_code ../../drone_code \
    --build-base ros_build --install-base ros_install
