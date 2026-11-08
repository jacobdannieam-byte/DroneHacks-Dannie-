#!/usr/bin/env bash
set -eo pipefail
cd /repository
tar --no-same-owner -xf /inputs/integrated.tar
source /opt/ros/humble/setup.bash
source /opt/px4_messages/install/setup.bash
export PYTHONPATH="/repository:$PYTHONPATH"
python3 /inputs/ros_timing_probe.py
python3 /inputs/ros_authority_probe.py
python3 /inputs/ros_recorder_probe.py
