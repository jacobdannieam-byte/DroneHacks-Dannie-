#!/usr/bin/env bash
set -eo pipefail
mode=$1; world=$2; seconds=$3; revision=$4; profile=$5; approval=$6
cd /repository
if [[ "$mode" == model_check || "$mode" == model_trial ]]; then
    [[ "$world" == drone_code/team_nav/course_maps/phone_scan_2026-11-06.json ]] || exit 2
    [[ "$profile" == scenarios/model_flight/h1_pinn_prefix.json ]] || exit 2
    : "${CALIBRATION_IMAGE_SHA256:?Pinned image required}"
    contract=scenarios/model_flight/h1_recorder.json
fi
if [[ "$mode" == calibration_check || "$mode" == calibration_trial ]]; then
    [[ "$world" == scenarios/calibration/straight_lane_v1.json ]] || exit 2
    : "${CALIBRATION_IMAGE_SHA256:?Pinned image identity required}"
    case "${profile##*/}" in
        c0_hover_land.json) contract=scenarios/calibration/c0_recorder.json ;;
        c1_stop_020.json) contract=scenarios/calibration/c1_recorder.json ;;
        r1_stationary_expiry.json) contract=scenarios/calibration/r1_recorder.json ;;
        *) echo 'Unknown calibration profile' >&2; exit 2 ;;
    esac
    python3 - "$profile" "$contract" <<'PY'
from hashlib import sha256
import json, sys
from pathlib import Path

profile, contract = (Path(path).read_bytes() for path in sys.argv[1:])
config, recorder = json.loads(profile), json.loads(contract)
if (recorder.get('profile_sha256') != sha256(profile).hexdigest()
        or recorder.get('trial_kind') != config['plan']['trial_kind']
        or recorder.get('clock_mapping') != config['clock_mapping']):
    raise SystemExit('Calibration profile and recorder contract differ')
PY
    export CALIBRATION_LANE_RUN=1
fi
# The host stages an immutable source archive, including pinned strategy models.
mkdir -p /evidence/ros /evidence/flight_records /evidence/gazebo_generated
mkdir -p simulator/generated_files
ln -s /evidence/flight_records simulator/generated_files/flight_records
ln -s /evidence/gazebo_generated simulator/gazebo_trial/generated
export SIMULATOR_WORLD_MAP="/repository/$world" ROS_LOG_DIR=/evidence/ros GAZEBO_ISOLATED_DDS=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export HYBRID_GAZEBO_RUN=1
[[ "$mode" != classical* ]] || export HYBRID_GAZEBO_RUN=0

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
if [[ "$mode" == hybrid_sitl || "$mode" == classical || "$mode" == classical_smoke ]]; then
    python3 -m examples.gazebo_container authorize "${auth[@]}"
    cp /approval.json /evidence/trial_approval.json
fi
if [ -n "$profile" ] && [[ "$mode" != calibration_* && "$mode" != model_* ]]; then
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
calibration_recorder_pid=
bag_root="$(mktemp -d)"
bag_path="$bag_root/observations"
required=()

start_recording() {
    # Delay the recorder and its deadlines until the stack has settled enough for DDS discovery.
    local attempt
    for ((attempt=0; attempt<150; attempt++)); do
        kill -0 "$stack_pid" || return 1
        if [ -f /evidence/stack.txt ] && grep -q 'Gazebo and PX4 are running' /evidence/stack.txt; then
            break
        fi
        sleep 1
    done
    ((attempt < 150)) || { echo "Gazebo startup timed out before recording"; return 1; }
    python3 -m simulator.gazebo_trial.recording_gate startup 30 /evidence/recording_startup.json "${required[@]}"
    cat > "$bag_root/storage.yaml" <<'YAML'
write:
  pragmas: ["journal_mode = WAL", "synchronous = NORMAL", "busy_timeout = 3000"]
YAML
    ros2 bag record --use-sim-time --storage-config-file "$bag_root/storage.yaml" -o "$bag_path" "$@" \
        > /evidence/rosbag.txt 2>&1 &
    bag_pid=$!
    BAG_PID="$bag_pid" python3 -m simulator.gazebo_trial.recording_gate writes "$bag_path" \
        /evidence/recording_ready.json "${required[@]}"
}

cleanup() {
    local status=$? bag_status=0 pid
    for pid in "$calibration_recorder_pid" "$bag_pid" "$stack_pid"; do
        [ -n "$pid" ] || continue
        kill -INT "$pid" 2>/dev/null || true
        for _ in {1..50}; do kill -0 "$pid" 2>/dev/null || break; sleep .1; done
        kill -TERM "$pid" 2>/dev/null || true
        wait "$pid" || true
    done
    if [ -d "$bag_path" ]; then
        # A Windows bind mount need not permit preserving Linux file times.
        cp -R "$bag_path" /evidence/observations || bag_status=1
        local minimum_duration=0
        if [[ "$mode" == observe || "$mode" == classical_observe || "$mode" == calibration_check ]]; then
            minimum_duration="$((seconds * 1000000000))"
        fi
        MIN_BAG_DURATION_NS="$minimum_duration" python3 -m simulator.gazebo_trial.recording_gate verify /evidence/observations \
            /evidence/recording_final.json "${required[@]}" || bag_status=1
    else
        bag_status=1
    fi
    exit "$((status != 0 ? status : bag_status))"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if [[ "$mode" == classical* ]]; then
    observe=false
    [ "$mode" != classical_observe ] || observe=true
    ln -s /evidence/smoke_flight_recordings simulator/generated_files/smoke_flight_recordings
    mkdir -p /evidence/smoke_flight_recordings
    mkdir -p /evidence/worlds
    ln -s /evidence/worlds simulator/generated_files/worlds
    ros2 launch /repository/simulator/gazebo_trial/full_route_in_gazebo.launch.py \
        observe_only:="$observe" > /evidence/stack.txt 2>&1 &
    stack_pid=$!
    required=(/clock /skybeta/state /visual_slam/tracking/odometry /visual_slam/status
        /fmu/out/vehicle_local_position /fmu/out/vehicle_odometry /fmu/out/vehicle_status_v1
        /fmu/out/vehicle_land_detected /fmu/out/estimator_status_flags /fmu/out/distance_sensor
        /simulator_only/oak/depth/image /simulator_only/oak/depth/camera_info)
    start_recording "${required[@]}" \
        /skybeta/setpoint/pose /skybeta/setpoint/twist /fmu/out/failsafe_flags \
        /fmu/in/manual_control_input /fmu/in/vehicle_visual_odometry /fmu/in/vehicle_command \
        /fmu/in/offboard_control_mode /fmu/in/trajectory_setpoint /fmu/out/vehicle_command_ack \
        /simulator_only/touches
    python3 -m simulator.gazebo_trial.classical_preflight "$observe" "$seconds" /evidence/classical_preflight.json
    kill -0 "$stack_pid" && kill -0 "$bag_pid" || exit 1
    [ "$observe" = false ] || exit 0
    result=0; score=0
    if [ "$mode" = classical_smoke ]; then
        timeout --signal=INT --kill-after=10 300 python3 simulator/test_flights/rise_and_land_test.py \
            --ros-args -p use_sim_time:=true || result=$?
    else
        timeout --signal=INT --kill-after=10 450 python3 simulator/test_flights/route_test.py \
            --drone-map "$world" --ros-args -p use_sim_time:=true || result=$?
    fi
    # Leave time for the independent referee and PX4 status to record touchdown and disarm.
    sleep 3
    python3 -m simulator.gazebo_trial.referee_result /evidence/flight_records || score=$?
    exit "$((result != 0 ? result : score))"
fi

ros2 launch /repository/simulator/gazebo_trial/hybrid_environment.launch.py > /evidence/stack.txt 2>&1 &
stack_pid=$!
topics="$(python3 -m examples.gazebo_container topics)"
mapfile -t bag_topics <<< "$topics"
required=("${bag_topics[@]}" /gazebo_hybrid/simulated_estimator/health_diagnostics /fmu/out/distance_sensor)
start_recording "${required[@]}" \
    /fmu/in/offboard_control_mode /fmu/in/trajectory_setpoint /fmu/in/vehicle_command /refnav/status \
    /qualification/events /calibration_trial/control /fmu/out/vehicle_command_ack /simulator_only/touches
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

# Capture effective settings without changing the deployed recovery behavior.
(
    cd /opt/PX4-Autopilot/build/px4_sitl_default/rootfs
    for parameter in COM_OF_LOSS_T COM_OBL_RC_ACT MPC_LAND_SPEED MPC_LAND_CRWL MPC_Z_VEL_MAX_DN MPC_Z_VEL_MAX_UP COM_DISARM_LAND; do
        ../bin/px4-param show "$parameter"
    done
) > /evidence/native_recovery_parameters.txt

ps -eo pid,comm,%cpu,rss > /evidence/resources_start.txt
case "$mode" in
    calibration_check|calibration_trial|model_check|model_trial)
        geometry=artifacts/taskforce_batch_003/native-lane-reviewed-v2/geometry-review.json
        contract_sha="$(sha256sum "$contract" | cut -d' ' -f1)"
        cp "$profile" /evidence/calibration_profile.json
        cp "$contract" /evidence/recorder_contract.json
        python3 -m examples.qualification_recorder --contract "$contract" --contract-sha256 "$contract_sha" \
            --output /evidence/calibration_recording.json --ready-output /evidence/calibration_recorder_ready.json \
            --seconds 240 > /evidence/calibration_recorder.txt 2>&1 &
        calibration_recorder_pid=$!
        for attempt in {1..100}; do
            kill -0 "$calibration_recorder_pid" && kill -0 "$bag_pid" || exit 1
            [ ! -f /evidence/calibration_recorder_ready.json ] || break
            sleep .05
        done
        [ -f /evidence/calibration_recorder_ready.json ] || exit 1
        owner_module=examples.calibration_trial_owner
        if [[ "$mode" == model_* ]]; then owner_module=examples.model_flight_owner; fi
        calibration_args=(--profile "$profile" --world simulator/gazebo_trial/generated/worlds/straight_lane_v1_full_simulator.sdf
            --vehicle simulator/gazebo_trial/generated/models/drone_16_like/model.sdf
            --geometry-evidence "$geometry" --revision "$revision" --image-sha256 "$CALIBRATION_IMAGE_SHA256")
        if [[ "$mode" == model_* ]]; then
            calibration_args=(--profile "$profile" --world simulator/gazebo_trial/generated/worlds/phone_scan_2026-11-06_full_simulator.sdf
                --vehicle simulator/gazebo_trial/generated/models/drone_16_like/model.sdf
                --revision "$revision" --image-sha256 "$CALIBRATION_IMAGE_SHA256")
        fi
        if [[ "$mode" == calibration_check || "$mode" == model_check ]]; then
            python3 -m "$owner_module" "${calibration_args[@]}" --disarmed-check \
                --output /evidence/calibration_disarmed_check.json
            sleep "$seconds"
            kill -0 "$calibration_recorder_pid" && kill -0 "$bag_pid" && kill -0 "$stack_pid"
        else
            [ -n "$approval" ] || exit 2
            calibration_args+=(--approval "$approval" --recording-contract "$contract"
                --recorder-ready /evidence/calibration_recorder_ready.json
                --recorder-pid "$calibration_recorder_pid" --bag-pid "$bag_pid")
            # No parameter mutation or command endpoint before exact approval.
            python3 -m "$owner_module" "${calibration_args[@]}" --authorize-only \
                --output /evidence/calibration_authorization.json
            (
                cd /opt/PX4-Autopilot/build/px4_sitl_default/rootfs
                ../bin/px4-param set MPC_LAND_SPEED 0.3
                ../bin/px4-param show MPC_LAND_SPEED
            ) > /evidence/approved_landing_speed.txt
            grep -Eq 'MPC_LAND_SPEED.*: 0\.3000' /evidence/approved_landing_speed.txt
            result=0
            owner_output=/evidence/calibration_owner.json
            [[ "$mode" != model_trial ]] || owner_output=/evidence/model_owner.json
            timeout --signal=INT --kill-after=5 130 python3 -m "$owner_module" \
                "${calibration_args[@]}" --output "$owner_output" || result=$?
            # Observe recovery for at most 60 s after the command owner exits.
            # Emergency simulator cancellation requires its own explicit approval
            # flag and can never masquerade as landing or authorize a retry.
            recovered=0
            recovery_deadline=$((SECONDS + 60))
            for recovery_attempt in {1..12}; do
                kill -0 "$stack_pid" && kill -0 "$bag_pid" || break
                remaining=$((recovery_deadline - SECONDS))
                [ "$remaining" -gt 0 ] || break
                if timeout --signal=INT --kill-after=2 "$remaining" python3 -m examples.gazebo_container probe --seconds 4 \
                    --output "/evidence/recovery-$recovery_attempt.json"; then
                    recovered=1
                    break
                fi
            done
            if [ "$recovered" != 1 ]; then
                printf '%s\n' '{"result":"FAIL","reason":"emergency_simulator_termination_without_confirmed_landing","landing_confirmed":false,"retry_authorized":false,"evidence":"recovery-*.json and observations/"}' \
                    > /evidence/emergency_simulator_termination.json
                result=1
            fi
            # The complete raw bag is copied by cleanup. Independent model audit
            # is run after container exit against the finalized bag.
            exit "$result"
        fi
        ;;
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
