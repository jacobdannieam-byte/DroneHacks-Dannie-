#!/usr/bin/env bash
set -eo pipefail
case "$2" in c1_stop_020|r1_stationary_expiry) ;; *) exit 2 ;; esac
mkdir -p /repository
cd /repository
tar --no-same-owner -xmf /source.tar
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ "$2" == r1_stationary_expiry ]]; then
    mkdir -p /evidence/operator-dry-run
    python3 -m examples.calibration_operator_capture --dry-run --output-dir /evidence/operator-dry-run \
        > /evidence/operator-dry-run.txt 2>&1
fi
bash simulator/gazebo_trial/inside_the_box/run_hybrid.sh calibration_check \
    scenarios/calibration/straight_lane_v1.json 30 "$1" "scenarios/calibration/$2.json" '' \
    > /evidence/launcher.txt 2>&1
cp /root/.gz/rendering/ogre2.log /evidence/ogre2.log
