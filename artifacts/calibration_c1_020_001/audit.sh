#!/usr/bin/env bash
set -o pipefail
mkdir -p /repository
cd /repository
tar --no-same-owner -xmf /source.tar
source /opt/ros/humble/setup.bash
source /opt/px4_messages/install/setup.bash
export PYTHONPATH="/repository${PYTHONPATH:+:$PYTHONPATH}"
python3 -m examples.calibration_bag_audit /raw/calibration_recording.json /raw/observations /output/calibration-bag-audit.json > /output/calibration-bag-audit.txt 2>&1
correlation=$?
python3 -m examples.stopping_trial_analysis /raw/calibration_recording.json /output/stopping-analysis.json > /output/stopping-analysis.txt 2>&1
analysis=$?
printf '{"raw_command_correlation_exit":%s,"stopping_analysis_exit":%s,"interpretation":"Failed C1 has no brake event; qualification rejection must remain preserved."}\n' "$correlation" "$analysis" > /output/standard-audits.json
