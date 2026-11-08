#!/usr/bin/env bash
set -eo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p /repository
cd /repository
tar --no-same-owner -xmf /source.tar
bash simulator/gazebo_trial/inside_the_box/run_hybrid.sh observe drone_code/team_nav/course_maps/phone_scan_2026-11-06.json 90 "$1" '' '' > /evidence/launcher.txt 2>&1 &
launcher=$!
for attempt in {1..150}; do
    kill -0 "$launcher" || { wait "$launcher"; exit 1; }
    test ! -f /evidence/disarmed-1.json || break
    sleep 1
done
source simulator/inside_the_box/load_ros_environment.sh
export PYTHONPATH="/repository${PYTHONPATH:+:$PYTHONPATH}"
python3 -m examples.build_disarmed_shadow_profiles --probe /evidence/disarmed-1.json --output-dir /evidence/spatial-shadow-profiles > /evidence/profile-build.txt 2>&1
for name in hybrid_existing hybrid_information_gain hybrid_learned_ranker hybrid_physics_informed; do
    result=0
    timeout --signal=INT --kill-after=5 25 python3 -m examples.hybrid_launch --config "/evidence/spatial-shadow-profiles/$name.json" --seconds 15 --output "/evidence/$name-shadow.json" > "/evidence/$name-shadow.txt" 2>&1 || result=$?
    printf '%s %s\n' "$name" "$result" >> /evidence/shadow-exit-codes.txt
done
wait "$launcher"
