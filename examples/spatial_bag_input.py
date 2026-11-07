"""Read a reviewed single-epoch Jazzy rosbag into the Phase 2 observation boundary."""

from dataclasses import dataclass
from pathlib import Path
import math

import numpy as np

from refnav.integration_timing import measure
from refnav.world_model.spatial_input import DepthCalibration, DepthCapture, PoseSample, SpatialReference
from refnav.world_model.spatial_input import decode_rectified_depth, stamp_ok
from refnav.world_model.spatial_pipeline import SpatialWorld

TOPICS = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/camera/depth/camera_info": "sensor_msgs/msg/CameraInfo",
    "/camera/depth/image_raw": "sensor_msgs/msg/Image",
    "/visual_slam/tracking/odometry": "nav_msgs/msg/Odometry",
    "/estimator/healthy": "std_msgs/msg/Bool",
    "/fmu/in/vehicle_visual_odometry": "px4_msgs/msg/VehicleOdometry",
}
CLOCK_SKEW_NS = 100_000_000
HEALTH_AGE_NS = 300_000_000
EVENT_LIMIT = 250_000


def _ns(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid ROS timestamp")
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class BagProfile:
    reference: SpatialReference
    calibration: DepthCalibration
    position_error_m: float
    angle_error_rad: float

    def __post_init__(self):
        if not all(math.isfinite(v) and 0 < v <= limit for v, limit in (
                (self.position_error_m, 1.), (self.angle_error_rad, .5))):
            raise ValueError("bag requires reviewed positive pose error bounds")

    @classmethod
    def from_dict(cls, value):
        if value.get("record_clock") != "ros_sim" or value.get("rectified") is not True:
            raise ValueError("bag profile must declare ROS simulation receipt time and rectified depth")
        return cls(SpatialReference(**value["reference"]), DepthCalibration(**value["calibration"]),
                   value["position_error_m"], value["angle_error_rad"])


def _calibration_matches(info, calibration):
    projection = tuple(info.p)
    if any(projection):
        intrinsics = (projection[0], projection[5], projection[2], projection[6])
        # Extrinsics describe the actual camera center. Do not silently apply a
        # virtual projection origin in addition to that extrinsic transform.
        if projection[3] != 0 or projection[7] != 0:
            return False
    else:
        intrinsics = (info.k[0], info.k[4], info.k[2], info.k[5])
        if any(info.d):
            return False
    return (info.header.frame_id == calibration.camera_frame
            and (info.width, info.height) == (calibration.width, calibration.height)
            and np.allclose(intrinsics, calibration.K, rtol=0, atol=1e-6))


def _covariance_consistent(values, profile):
    matrix = np.asarray(values, dtype=float).reshape(6, 6)
    if (not np.isfinite(matrix).all() or not np.allclose(matrix, matrix.T, rtol=0, atol=1e-8)
            or np.linalg.eigvalsh(matrix)[0] < -1e-10):
        return False
    # These are sanity checks against independently reviewed error bounds.
    # Covariance alone does not establish a physical worst-case bound.
    for part, bound in ((matrix[:3, :3], profile.position_error_m), (matrix[3:, 3:], profile.angle_error_rad)):
        if np.any(np.diag(part) <= 0) or np.linalg.eigvalsh(part)[-1] > bound ** 2:
            return False
    return True


class RecordedSpatialReplay:
    """Bounded message normalizer shared by the bag reader and offline tests."""

    topics = TOPICS

    def __init__(self, profile, *, timings=None):
        self.profile = profile
        self.timings = timings
        self.world = SpatialWorld(profile.reference, profile.calibration, timings=timings)
        self.rows = []
        self.now = self._clock = self._health = self._ev_reset = self._ev_received = None
        self._pose_capture = self._pending = None
        self._info_ok = self._clock_progress = False

    def _advance(self, topic, received_ns):
        if topic not in self.topics or not stamp_ok(received_ns) or (self.now is not None and received_ns < self.now):
            raise ValueError("unexpected topic or backward bag receipt time")
        self.now = received_ns

    def feed(self, topic, msg, received_ns):
        self._advance(topic, received_ns)
        if topic == "/clock":
            clock = _ns(msg.clock)
            if (self._clock is not None and clock < self._clock) or abs(clock - received_ns) > CLOCK_SKEW_NS:
                raise ValueError("clock rewind or mismatched recording clock. Replay one reviewed epoch at a time")
            self._clock_progress |= self._clock is not None and clock > self._clock
            self._clock = clock
            return
        self._require_clock(received_ns)

        if topic == "/camera/depth/camera_info":
            if not _calibration_matches(msg, self.profile.calibration):
                raise ValueError("recorded CameraInfo differs from the reviewed depth calibration")
            self._info_ok = True
        elif topic == "/fmu/in/vehicle_visual_odometry":
            if self._ev_reset is not None and msg.reset_counter != self._ev_reset:
                raise ValueError("EV reset crosses the selected bag epoch")
            self._ev_reset = int(msg.reset_counter)
            self._ev_received = received_ns
        elif topic == "/estimator/healthy":
            self._health = (received_ns, bool(msg.data))
            if not msg.data:
                self.world.push_pose(PoseSample(self.profile.reference, received_ns, received_ns, False))
                self._pose_capture = max(received_ns, self._pose_capture or 0)
        elif topic == "/visual_slam/tracking/odometry":
            self._feed_pose(msg, received_ns)
        else:
            self._flush(force=True)
            if not self._info_ok or not self._clock_progress or self._ev_reset is None:
                raise ValueError("depth needs prior CameraInfo, an advancing /clock and EV epoch evidence")
            if msg.header.frame_id != self.profile.calibration.camera_frame:
                raise ValueError("depth optical frame differs from reviewed calibration")
            depth = measure(self.timings, "depth_normalization", decode_rectified_depth,
                            msg.data, encoding=msg.encoding, width=int(msg.width),
                            height=int(msg.height), step=int(msg.step), bigendian=bool(msg.is_bigendian))
            stamp = _ns(msg.header.stamp)
            self._pending = DepthCapture(self.profile.reference, stamp, received_ns, msg.header.frame_id,
                                         f"depth-{stamp}", depth)
        self._flush()

    def _require_clock(self, received_ns):
        if self._clock is None or abs(received_ns - self._clock) > CLOCK_SKEW_NS:
            raise ValueError("bag record time is unbound to the observed simulation clock")

    def _feed_pose(self, msg, received_ns):
        ref = self.profile.reference
        if msg.header.frame_id != ref.frame_id or msg.child_frame_id != ref.body_frame:
            raise ValueError("odometry frames differ from reviewed bag reference")
        stamp = _ns(msg.header.stamp)
        healthy = (self._health is not None and self._health[1] and self._ev_received is not None
                   and received_ns - self._ev_received <= HEALTH_AGE_NS
                   and 0 <= stamp - self._health[0] <= HEALTH_AGE_NS
                   and _covariance_consistent(msg.pose.covariance, self.profile))
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        sample = PoseSample(ref, stamp, received_ns, bool(healthy), (p.x, p.y, p.z),
                            (q.w, q.x, q.y, q.z), self.profile.position_error_m, self.profile.angle_error_rad)
        self.world.push_pose(sample)
        self._pose_capture = max(stamp, self._pose_capture or 0)

    def _flush(self, force=False):
        frame = self._pending
        if frame is None or (not force and (self._pose_capture is None or self._pose_capture < frame.capture_ns)):
            return
        if len(self.rows) >= 2048:
            raise ValueError("bag exceeds the 2048-depth-frame replay budget")
        self._pending = None
        result = self.world.process(frame, now_ns=self.now)
        self.rows.append({"capture_ns": frame.capture_ns, "received_ns": frame.received_ns,
                          "processed_ns": self.now, "accepted": result.accepted, "reason": result.reason,
                          "pose_bracket_ns": result.pose_bracket_ns, "semantic_events": result.semantic_events})

    def finish(self):
        self._flush(force=True)
        if not self.rows:
            raise ValueError("bag has no depth observations")
        return self.world.snapshot(now_ns=self.now)


def read_bag(path: Path, replay):
    """Consume canonical ROS topics without spinning ROS or publishing anything."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id=""), ConverterOptions("", ""))
    try:
        types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
        if any(types.get(topic) != kind for topic, kind in replay.topics.items()):
            raise ValueError("bag lacks one or more required native topic/message pairs")
        reader.set_filter(StorageFilter(topics=list(replay.topics)))
        classes = {topic: get_message(kind) for topic, kind in replay.topics.items()}
        count = 0
        while reader.has_next():
            topic, data, received_ns = reader.read_next()
            count += 1
            if count > EVENT_LIMIT or len(data) > 2_000_000:
                raise ValueError("bag exceeds event or serialized payload budget")
            replay.feed(topic, deserialize_message(data, classes[topic]), received_ns)
    finally:
        reader.close()
    return replay.finish()
