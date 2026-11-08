#!/usr/bin/env bash
set -eo pipefail
[[ "$1" == 2ecbee3281727cff8c3791068f3503e66edf2bcf && "$2" == c1_stop_020 ]] || exit 2
mkdir -p /repository
cd /repository
tar --no-same-owner -xmf /source.tar
cp /approval.json /evidence/trial_approval.json
trap 'cp /root/.gz/rendering/ogre2.log /evidence/ogre2.log 2>/dev/null || true' EXIT
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
bash simulator/gazebo_trial/inside_the_box/run_hybrid.sh calibration_trial \
    scenarios/calibration/straight_lane_v1.json 30 "$1" scenarios/calibration/c1_stop_020.json /approval.json \
    > /evidence/launcher.txt 2>&1
