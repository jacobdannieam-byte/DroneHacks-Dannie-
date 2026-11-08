"""Synthetic calibration and private ROS event delivery. Creates no actuator endpoints."""

import json
import os
from time import monotonic
from uuid import uuid4

from examples.h1_observation import ClockMapping
from examples.qualification_recorder import TrialRecording, event_publisher_identity
from refnav.calibration_trial import CalibrationPlan, CalibrationTrial, Observation
from refnav.core import NAV_AUTO_LAND, NAV_OFFBOARD


class Fixture:
    def __init__(self, trial_kind='stopping'):
        self.mapping = ClockMapping('fixture_px4', 'fixture_ros', 0, 0, 1, 30_000_000_000, 'synthetic_fixture')
        self.trial = CalibrationTrial(CalibrationPlan(
            trial_kind, (0., 0., 0.), (-.5, -.5, -1.25), (4., .5, .02), .1, 1., .2, 1.5,
            .2, .2, .5, 12., 60., 120., 3.))
        self.ros_ns = self.wall_ns = 1_000_000_000
        self.outputs = []

    def step(self, *, position=(0., 0., 0.), velocity=(0., 0., 0.),
             nav=0, armed=False, landed=True, admitted=True, publish=True):
        self.ros_ns += 100_000_000
        self.wall_ns += 100_000_000
        state = Observation(position, velocity, self.ros_ns, self.ros_ns, armed, landed, nav)
        intent = self.trial.step(self.ros_ns, self.wall_ns, state, admitted=admitted)
        if intent is not None:
            self.outputs.append(intent)
            if publish:
                self.trial.published(intent, self.ros_ns, self.mapping.source_stamp_us(self.ros_ns), wall_ns=self.wall_ns)
        return intent

    def reach_hover(self):
        for _ in range(11):
            self.step()
        assert self.outputs[-1].command == 'offboard'
        assert self.step(nav=NAV_OFFBOARD).command == 'arm'
        assert self.step(nav=NAV_OFFBOARD, armed=True).velocity_ned_mps == (0., 0., -.2)
        for _ in range(6):
            self.step(position=(0., 0., -1.), nav=NAV_OFFBOARD, armed=True, landed=False)

    def start_transit(self):
        self.reach_hover()
        assert self.outputs[-1].velocity_ned_mps == (.2, 0., 0.)

    def complete(self):
        self.start_transit()
        for north in (.3, .6, .9, 1.2, 1.5):
            self.step(position=(north, 0., -1.), velocity=(.2, 0., 0.),
                      nav=NAV_OFFBOARD, armed=True, landed=False)
        assert self.outputs[-1].event == 'brake_command'
        for _ in range(6):
            self.step(position=(1.6, 0., -1.), nav=NAV_OFFBOARD, armed=True, landed=False)
        assert self.outputs[-1].command == 'land'
        self.step(position=(1.6, 0., -.6), nav=NAV_AUTO_LAND, armed=True, landed=False)
        self.step(position=(1.6, 0., 0.), nav=NAV_AUTO_LAND, armed=False)
        assert self.trial.result == 'complete'


def main():
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not domain.isdecimal() or not 1 <= int(domain) <= 232:
        raise SystemExit('Use a dedicated local-only ROS domain')
    import rclpy
    from std_msgs.msg import String
    from onboard.diagnostics.sitl_graph import inspect

    fixture = Fixture()
    fixture.complete()
    identity = dict(trial_id='private_fixture', owner_id='/private_calibration_owner', profile_sha256='a' * 64)
    record = TrialRecording(fixture.mapping.ros_clock_id, 'px4_local_ned',
                            clock_mapping=fixture.mapping, **identity)
    topic = '/qualification/calibration_' + uuid4().hex[:10]
    rclpy.init(args=[])
    node = rclpy.create_node(identity['owner_id'][1:])
    try:
        pub = node.create_publisher(String, topic, 10)

        def receive(message):
            publisher = event_publisher_identity(node, topic, identity['owner_id'])
            record.event(message.data, json.loads(message.data)['stamp_ns'], publisher=publisher)

        node.create_subscription(String, topic, receive, 10)
        until = monotonic() + 3.
        while pub.get_subscription_count() != 1 and monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        assert pub.get_subscription_count() == 1
        for event in fixture.trial.events:
            pub.publish(String(data=json.dumps(dict(event, **identity))))
        while len(record.document['events']) < 2 and monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        assert not record.faults and [e['event'] for e in record.document['events']] == ['brake_command', 'land_command']
        assert not record.result()['recording_complete']  # No native disarm was fabricated.
        assert not inspect(node, strict_motion=True)['issues']
        print(json.dumps(dict(scope='synthetic driver and private event DDS transport', events=record.document['events'],
                              native_command_publishers=0, measured_stopping_evidence=False)))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
