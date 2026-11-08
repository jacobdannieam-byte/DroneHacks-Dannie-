"""R1 steady-timer rehearsal with private synthetic topics and no actuator publishers."""

from collections import deque
from dataclasses import asdict
from hashlib import sha256
import json
import os
from time import monotonic_ns
from uuid import uuid4

from examples.calibration_recovery_audit import validate_operator_receipt
from examples.calibration_trial_owner import load_profile
from onboard.diagnostics.sitl_graph import inspect
from refnav.calibration_trial import CalibrationTrial, Observation, NS, RECOVERY_LEASE_NS, ZERO
from refnav.core import NAV_OFFBOARD


def main():
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not domain.isdecimal() or not 1 <= int(domain) <= 232:
        raise SystemExit('Use a dedicated local-only ROS domain in an isolated container')

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import SingleThreadedExecutor
    from rosgraph_msgs.msg import Clock as ClockMessage
    from std_msgs.msg import String

    _, plan, mapping = load_profile('scenarios/calibration/r1_stationary_expiry.json')
    trial = CalibrationTrial(plan)
    private = '/qualification/r1_' + uuid4().hex[:12]
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-r', f'/clock:={private}/clock'])
    executor = SingleThreadedExecutor()
    fixture = owner = None
    try:
        fixture = rclpy.create_node('r1_private_fixture', use_global_arguments=False)
        owner = rclpy.create_node('r1_private_owner')
        executor.add_node(fixture)
        executor.add_node(owner)
        clock_pub = fixture.create_publisher(ClockMessage, private + '/clock', 10)
        observation_pub = fixture.create_publisher(String, private + '/observation', 10)
        receipt_pub = fixture.create_publisher(String, private + '/operator_receipt', 10)
        output_pub = owner.create_publisher(String, private + '/intent_diagnostic', 10)
        observations = deque(maxlen=12)
        outputs, receipts = [], []

        def observe(message):
            observations.append(Observation(**json.loads(message.data)))

        owner.create_subscription(String, private + '/observation', observe, 10)
        owner.create_subscription(String, private + '/operator_receipt', lambda m: receipts.append(json.loads(m.data)), 10)
        fixture.create_subscription(String, private + '/intent_diagnostic', lambda m: outputs.append(json.loads(m.data)), 10)

        def wait_for(predicate, detail, seconds=5):
            deadline = monotonic_ns() + seconds * NS
            while not predicate():
                if monotonic_ns() >= deadline:
                    raise RuntimeError('Timed out waiting for ' + detail)
                executor.spin_once(timeout_sec=.01)

        wait_for(lambda: all(p.get_subscription_count() == 1 for p in
                            (clock_pub, observation_pub, receipt_pub, output_pub)), 'private endpoint discovery')
        assert not inspect(fixture, strict_motion=True)['issues']
        start = monotonic_ns()

        def feed():
            stamp = NS + monotonic_ns() - start
            message = ClockMessage()
            message.clock.sec, message.clock.nanosec = divmod(stamp, NS)
            clock_pub.publish(message)
            lifted = trial.phase in ('climb', 'authority_expiry', 'done')
            armed = lifted or trial.phase == 'arm'
            nav = 2 if trial.phase == 'done' else NAV_OFFBOARD if armed or trial.phase == 'mode' else 0
            sample = Observation((0., 0., -1. if lifted else 0.), ZERO, stamp, stamp, armed, not lifted, nav)
            observation_pub.publish(String(data=json.dumps(asdict(sample))))

        def tick():
            now, wall = owner.get_clock().now().nanoseconds, monotonic_ns()
            sample = next((row for row in reversed(observations)
                           if 0 <= now - row.position_stamp_ns <= 100_000_000), None)
            intent = trial.step(now, wall, sample, admitted=True)
            if intent is None:
                return
            stamp = mapping.source_stamp_us(now)
            output_pub.publish(String(data=json.dumps(dict(asdict(intent), ros_ns=now, wall_ns=wall, px4_issue_us=stamp))))
            trial.published(intent, now, stamp, wall_ns=wall)

        fixture.create_timer(.02, feed, clock=Clock(clock_type=ClockType.STEADY_TIME))
        owner.create_timer(.05, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        wait_for(lambda: trial.result is not None, 'stationary expiry')
        assert trial.result == 'planned_authority_expiry'
        lease = trial.recovery_lease
        assert lease['deadline_wall_ns'] - lease['last_output_wall_ns'] == RECOVERY_LEASE_NS
        assert lease['expiry_detected_wall_ns'] >= lease['deadline_wall_ns']
        wait_for(lambda: any(out['recovery_lease'] for out in outputs), 'final diagnostic target delivery')
        final = outputs[-1]
        assert final['recovery_lease'] and final['velocity_ned_mps'] == list(ZERO) and final['command'] is None
        assert sum(out['recovery_lease'] for out in outputs) == 1
        assert [out['command'] for out in outputs if out['command']] == ['offboard', 'arm']
        assert final['px4_issue_us'] == lease['last_output_px4_us']
        before = len(outputs)
        wait_for(lambda: owner.get_clock().now().nanoseconds >= lease['expiry_detected_ros_ns'] + 250_000_000
                 and observations and observations[-1].nav_state == 2, 'clock advance after yield')
        assert len(outputs) == before and not inspect(fixture, strict_motion=True)['issues']

        sample = observations[-1]
        fixture_record = dict(trial_id='private_fixture_r1', recovery_operator='private_fixture_operator', recovery_lease=lease)
        fixture_probe = json.dumps(dict(synthetic=True, observation=asdict(sample)), sort_keys=True).encode()
        fixture_console = b'SYNTHETIC FIXTURE ONLY - no PX4 console command executed\n'
        handoff = dict(operator_preflight_sha256=sha256(fixture_probe).hexdigest(),
                       console_log_sha256=sha256(fixture_console).hexdigest())
        fixture_receipt = dict(synthetic=True, trial_id=fixture_record['trial_id'],
            operator=fixture_record['recovery_operator'], command='px4-commander land', exit_code=0,
            owner_exited=True, idle_motion_graph=True, fresh_healthy_in_lane=True,
            status_px4_us=mapping.source_stamp_us(sample.authority_stamp_ns),
            land_issue_ros_ns=owner.get_clock().now().nanoseconds, land_issue_wall_ns=monotonic_ns(), **handoff)
        receipt_pub.publish(String(data=json.dumps(fixture_receipt)))
        receipt_pub.publish(String(data=json.dumps(dict(fixture_receipt, console_log_sha256='0' * 64))))
        wait_for(lambda: len(receipts) == 2, 'operator receipt schema fixtures')
        validate_operator_receipt(fixture_record, receipts[0], handoff=handoff)
        try:
            validate_operator_receipt(fixture_record, receipts[1], handoff=handoff)
        except ValueError:
            pass
        else:
            raise AssertionError('Mismatched fixture console digest accepted')

        print(json.dumps(dict(scope='private ROS timers and synthetic telemetry/operator receipt',
            native_command_publishers=0, console_command_executed=False, final_intent=final,
            diagnostic_output_count=before, outputs_after_final=0, lease=lease,
            valid_receipt_schema=True, mismatched_console_digest_rejected=True)))
    finally:
        executor.shutdown()
        if owner is not None:
            owner.destroy_node()
        if fixture is not None:
            fixture.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
