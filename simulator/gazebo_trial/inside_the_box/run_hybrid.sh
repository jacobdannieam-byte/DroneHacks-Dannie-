#!/usr/bin/env bash
set -eo pipefail
mode=$1; world=$2; seconds=$3; revision=$4; profile=$5; approval=$6
cd /repository
# Copy only code/assets. Builds and generated files write to this run's evidence tree.
tar -C /source --exclude=__pycache__ --exclude=simulator/gazebo_trial/generated --exclude=simulator/generated_files \
    -cf - simulator drone_code examples refnav onboard scenarios | tar --no-same-owner -xf -
mkdir -p /evidence/ros /evidence/flight_records /evidence/gazebo_generated
mkdir -p simulator/generated_files
ln -s /evidence/flight_records simulator/generated_files/flight_records
ln -s /evidence/gazebo_generated simulator/gazebo_trial/generated
export SIMULATOR_WORLD_MAP="/repository/$world" ROS_LOG_DIR=/evidence/ros GAZEBO_ISOLATED_DDS=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export HYBRID_GAZEBO_RUN=1
[ "$mode" != classical ] || export HYBRID_GAZEBO_RUN=0

timeout --kill-after=10 600 bash simulator/inside_the_box/build_our_packages.sh > /evidence/colcon.txt 2>&1
source simulator/inside_the_box/load_ros_environment.sh
export PYTHONPATH="/repository${PYTHONPATH:+:$PYTHONPATH}"
{
    uname -m
    gz sim --versions || gz sim --version || true
    printenv ROS_DISTRO ROS_DOMAIN_ID
    git -C /opt/PX4-Autopilot rev-parse HEAD
    git -C /opt/px4_messages/src/px4_msgs rev-parse HEAD
} > /evidence/versions.txt 2>&1
auth=(--mode "$mode" --world "$world" --revision "$revision")
[ -z "$profile" ] || auth+=(--profile "$profile")
[ -z "$approval" ] || auth+=(--approval "$approval")
if [[ "$mode" == hybrid_sitl || "$mode" == classical ]]; then
    python3 -m examples.gazebo_container authorize "${auth[@]}"
    cp /approval.json /evidence/trial_approval.json
fi
if [ -n "$profile" ]; then
    python3 - "$profile" <<'PY'
import json, shutil, sys
from pathlib import Path
profile = Path(sys.argv[1])
data = json.loads(profile.read_text())
for role, source in (("launcher", profile), ("h1", profile.parent / data["h1_profile"]),
                     ("course", profile.parent / data["mission_profile"])):
    shutil.copyfile(source, "/evidence/" + role + "_profile.json")
PY
fi

stack_pid=
bag_pid=
cleanup() {
    if [ -n "$bag_pid" ]; then kill -INT "$bag_pid" 2>/dev/null || true; fi
    if [ -n "$stack_pid" ]; then kill -INT "$stack_pid" 2>/dev/null || true; fi
    for pid in "$bag_pid" "$stack_pid"; do
        [ -n "$pid" ] || continue
        for _ in {1..50}; do kill -0 "$pid" 2>/dev/null || break; sleep .1; done
        kill -TERM "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if [ "$mode" = classical ]; then
    ros2 launch /repository/simulator/gazebo_trial/full_route_in_gazebo.launch.py > /evidence/stack.txt 2>&1 &
    stack_pid=$!
    # Imported test performs its own readiness and route checks.
    result=0; score=0
    timeout --signal=INT --kill-after=10 450 python3 simulator/test_flights/route_test.py --drone-map "$world" || result=$?
    python3 -m simulator.gazebo_trial.referee_result /evidence/flight_records || score=$?
    exit "$((result != 0 ? result : score))"
fi

ros2 launch /repository/simulator/gazebo_trial/hybrid_environment.launch.py > /evidence/stack.txt 2>&1 &
stack_pid=$!
topics="$(python3 -m examples.gazebo_container topics)"
mapfile -t bag_topics <<< "$topics"
ros2 bag record --use-sim-time -o /evidence/observations "${bag_topics[@]}" \
    /fmu/in/offboard_control_mode /fmu/in/trajectory_setpoint /fmu/in/vehicle_command /refnav/status \
    > /evidence/rosbag.txt 2>&1 &
bag_pid=$!
# Give DDS discovery / EKF bootstrap time without creating any owner. Every
# attempt leaves its report, and the entire startup has a bounded wall budget.
ready=0
for attempt in {1..12}; do
    kill -0 "$stack_pid" && kill -0 "$bag_pid" || { echo "Gazebo stack or recorder exited"; exit 1; }
    if python3 -m examples.gazebo_container probe --seconds 8 --output "/evidence/disarmed-$attempt.json"; then
        ready=1
        break
    fi
done
if [ "$ready" != 1 ]; then echo "Disarmed Gazebo sensor/authority preflight failed"; exit 1; fi

ps -eo pid,comm,%cpu,rss > /evidence/resources_start.txt
case "$mode" in
    observe)
        sleep "$seconds"
        kill -0 "$stack_pid" && kill -0 "$bag_pid"
        ;;
    hybrid_shadow)
        python3 -m examples.gazebo_container verify-shadow --profile "$profile"
        timeout --signal=INT --kill-after=5 "$((seconds + 10))" \
            python3 -m examples.hybrid_launch --config "$profile" --seconds "$seconds" --output /evidence/shadow.json
        ;;
    hybrid_sitl)
        result=0; score=0
        timeout --signal=INT --kill-after=10 450 python3 -m examples.gazebo_container fly "${auth[@]}" \
            --output /evidence/mission.json || result=$?
        # A stopped owner is not proof of landing. Keep the simulator and
        # recorder alive briefly to capture the existing PX4 failsafe.
        if [ "$result" != 0 ]; then sleep 15; fi
        python3 -m simulator.gazebo_trial.referee_result /evidence/flight_records || score=$?
        ps -eo pid,comm,%cpu,rss > /evidence/resources_end.txt
        exit "$((result != 0 ? result : score))"
        ;;
    *) exit 2 ;;
esac
ps -eo pid,comm,%cpu,rss > /evidence/resources_end.txt
