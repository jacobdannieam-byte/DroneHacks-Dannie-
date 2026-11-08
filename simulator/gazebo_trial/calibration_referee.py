"""Independent SIMULATOR-ONLY lane observer. No course score or flight commands.

Groundtruth is recorder/evaluator input only. A conservative sphere crossing a
boundary is a geometric intrusion, NOT a measured Gazebo contact. The current
world has no contact-sensor stream; that limitation is explicit in every summary.
"""
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import time

LANE_RELATIVE_PATH = 'scenarios/calibration/straight_lane_v1.json'
LANE_SHA256 = 'a5dd0fc8dee3226d3acafad7c53cfb1aa58bbef43f2e83f7837411b31d8266a7'
ROOT = Path(__file__).resolve().parents[2]
BODY_RADIUS_M = .35  # Reviewed design sphere; not a real-aircraft measurement.


def calibration_observer_enabled(environment=None, root=ROOT):
    """An explicit flag plus exact path AND bytes is required to replace referee."""
    env = os.environ if environment is None else environment
    flag = env.get('CALIBRATION_LANE_RUN', '0')
    if flag == '0':
        return False
    if flag != '1':
        raise ValueError('CALIBRATION_LANE_RUN must be 0 or 1')
    selected = Path(env.get('SIMULATOR_WORLD_MAP', '')).resolve()
    expected = (Path(root) / LANE_RELATIVE_PATH).resolve()
    if selected != expected or not selected.is_file() or sha256(selected.read_bytes()).hexdigest() != LANE_SHA256:
        raise ValueError('calibration observer requires exact reviewed lane path and SHA256')
    return True


def geometry_sample(position_ned, velocity_ned):
    """Pure sampled diagnostic under the reviewed Gazebo ENU/native NED binding."""
    position, velocity = tuple(position_ned), tuple(velocity_ned)
    if len(position) != 3 or len(velocity) != 3 or not all(math.isfinite(v) for v in (*position, *velocity)):
        raise ValueError('invalid truth telemetry')
    x, y, z = position[1], position[0], -position[2]
    margins = {'west': x + 3 - BODY_RADIUS_M, 'east': 3 - x - BODY_RADIUS_M,
               'south': y + 2 - BODY_RADIUS_M, 'north': 6 - y - BODY_RADIUS_M,
               'roof': 3.008 - z - BODY_RADIUS_M, 'floor': z + .04 - BODY_RADIUS_M}
    return {'position_ned_m': position, 'velocity_ned_mps': velocity,
            'position_world_enu_m': (x, y, z), 'speed_3d_mps': math.hypot(*velocity),
            'body_sphere_margin_m': margins,
            'side_or_roof_envelope_intrusion': any(value <= 0 for name,value in margins.items() if name != 'floor'),
            'floor_envelope_overlap': margins['floor'] <= 0,
            'center_below_modeled_floor': z < -.04,
            'navigation_center_box': -.5 <= x <= .5 and -.5 <= y <= 4 and .75 <= z <= 1.25}


def main():
    if not calibration_observer_enabled():
        raise RuntimeError('explicit calibration lane selection required')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleStatus, VehicleLandDetected
    from std_msgs.msg import String
    from onboard.diagnostics.phase1_probe import px4_topic
    from course_simulator.paths import FOLDER_FOR_FLIGHT_RECORDS

    class CalibrationObserver(Node):
        def __init__(self):
            super().__init__('calibration_lane_observer')
            directory = Path(FOLDER_FOR_FLIGHT_RECORDS)
            directory.mkdir(parents=True, exist_ok=True)
            prefix = 'calibration_' + str(time.time_ns())
            self.summary_path = directory / (prefix + '_summary.json')
            self.samples = (directory / (prefix + '_truth.jsonl')).open('x', encoding='utf-8')
            self.latest = {}
            self.last_position_stamp = None
            self.intruding = False
            self.summary = {'scope': 'isolated_calibration_lane_only', 'course_score': None,
                            'policy_positive_clearance': False, 'flight_qualified': False,
                            'map_sha256': LANE_SHA256, 'contact_measurement_available': False,
                            'geometry_kind': 'sampled_conservative_body_sphere_not_actual_contact',
                            'frame_contract': 'Gazebo ENU=(native NED y,x,-z); floor z=-0.04; no recentering',
                            'samples': 0, 'intrusion_samples': 0, 'speed_exceedance_samples': 0,
                            'max_speed_3d_mps': None, 'latest_telemetry_fresh': False}
            self.touch = self.create_publisher(String, '/simulator_only/touches', 10)
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_groundtruth',
                                     lambda m: self.receive('position', m), qos_profile_sensor_data)
            self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude_groundtruth',
                                     lambda m: self.receive('attitude', m), qos_profile_sensor_data)
            self.create_subscription(VehicleStatus, px4_topic('vehicle_status', VehicleStatus),
                                     lambda m: self.receive('status', m), qos_profile_sensor_data)
            self.create_subscription(VehicleLandDetected, px4_topic('vehicle_land_detected', VehicleLandDetected),
                                     lambda m: self.receive('landed', m), qos_profile_sensor_data)
            self.create_timer(1 / 30, self.tick)
            self.create_timer(1., self.write_summary)
            self.write_summary()

        def receive(self, role, message):
            self.latest[role] = (message, self.get_clock().now().nanoseconds, time.monotonic_ns())

        def write_summary(self):
            self.summary_path.write_text(json.dumps(self.summary, indent=2, allow_nan=False) + '\n', encoding='utf-8')
            self.samples.flush()

        def tick(self):
            wall = time.monotonic_ns()
            limits = {'position': 300_000_000, 'attitude': 300_000_000,
                      'status': 750_000_000, 'landed': 1_500_000_000}
            fresh = all(role in self.latest and 0 <= wall-self.latest[role][2] <= limit
                        for role,limit in limits.items())
            self.summary['latest_telemetry_fresh'] = fresh
            if 'position' not in self.latest:
                return
            message, receipt, _ = self.latest['position']
            if message.timestamp == self.last_position_stamp:
                return
            self.last_position_stamp = message.timestamp
            try:
                row = geometry_sample((message.x, message.y, message.z), (message.vx, message.vy, message.vz))
            except (ValueError, TypeError):
                self.summary['invalid_truth_sample'] = True
                self.write_summary()
                return
            row.update(source_publish_us=int(message.timestamp),
                       source_sample_us=int(getattr(message, 'timestamp_sample', message.timestamp)),
                       receipt_ros_ns=receipt, observer_ros_ns=self.get_clock().now().nanoseconds,
                       receipt_monotonic_ns=self.latest['position'][2], telemetry_fresh=fresh,
                       source_reports={})
            for role,(sample, ros_ns, wall_ns) in self.latest.items():
                row['source_reports'][role] = {'source_publish_us': int(sample.timestamp),
                                               'receipt_ros_ns': ros_ns, 'receipt_monotonic_ns': wall_ns}
            if 'attitude' in self.latest:
                row['attitude_q_wxyz'] = [float(v) for v in self.latest['attitude'][0].q]
            row['armed'] = (self.latest['status'][0].arming_state == VehicleStatus.ARMING_STATE_ARMED
                            if 'status' in self.latest else None)
            row['landed'] = bool(self.latest['landed'][0].landed) if 'landed' in self.latest else None
            row['nav_state'] = int(self.latest['status'][0].nav_state) if 'status' in self.latest else None
            self.samples.write(json.dumps(row, allow_nan=False) + '\n')
            self.summary['samples'] += 1
            self.summary['intrusion_samples'] += int(row['side_or_roof_envelope_intrusion'])
            self.summary['speed_exceedance_samples'] += int(row['speed_3d_mps'] > .5)
            self.summary['max_speed_3d_mps'] = max(self.summary['max_speed_3d_mps'] or 0., row['speed_3d_mps'])
            self.summary['latest'] = row
            if row['side_or_roof_envelope_intrusion'] and not self.intruding:
                self.touch.publish(String(data='Calibration body-envelope boundary intrusion; geometric diagnostic, not measured contact'))
            self.intruding = row['side_or_roof_envelope_intrusion']

    rclpy.init()
    node = CalibrationObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_summary()
        node.samples.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
