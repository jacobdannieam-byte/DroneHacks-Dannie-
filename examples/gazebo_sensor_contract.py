"""Pure provenance and clock contracts for the imported Gazebo depth rig."""

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

from examples.spatial_bag_input import _ns
from refnav.world_model.spatial_input import NS


SIM_ESTIMATOR = "simulated_estimator"
SIM_ODOM = "/gazebo_hybrid/simulated_estimator/odometry"
SIM_STATUS = "/gazebo_hybrid/simulated_estimator/status"
SIM_PROCESSING = "/gazebo_hybrid/simulated_estimator/processing_status"
SIM_HEALTH = "/gazebo_hybrid/simulated_estimator/healthy"
SIM_DEPTH = "/simulator_only/oak/depth/image"
SIM_INFO = "/simulator_only/oak/depth/camera_info"
SIM_BINDINGS = {"/camera/depth/image_raw": SIM_DEPTH, "/camera/depth/camera_info": SIM_INFO,
                "/visual_slam/tracking/odometry": SIM_ODOM, "/visual_slam/processing_status": SIM_PROCESSING,
                "/estimator/healthy": SIM_HEALTH}
OPTICAL_FRAME = "oak_depth_optical_frame"
OPTICAL_TO_FLU = ((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.))
TRACKING_MAX_AGE_NS = 200_000_000


@dataclass(frozen=True)
class CameraRig:
    translation: tuple
    width: int
    height: int
    rate: float
    near_m: float
    far_m: float


def read_camera_rig(models: Path) -> CameraRig:
    """Read only the simulator model's authored mount and lens poses."""
    body = ET.parse(models / "drone_16_with_oak_d_s2/model.sdf").getroot()
    camera = ET.parse(models / "oak_d_s2_camera/model.sdf").getroot()
    mount, = [node for node in body.findall("./model/include") if node.findtext("uri") == "model://oak_d_s2_camera"]
    sensor, = camera.findall("./model/link[@name='camera_link']/sensor[@type='depth_camera']")
    lens = sensor.find("camera")

    def forward(node):
        values = tuple(map(float, node.findtext("pose").split()))
        if len(values) != 6 or values[1:] != (0.,) * 5 or not 0 <= values[0] <= .5:
            raise ValueError("Gazebo mount changed: review the complete optical transform")
        return values[0]

    if lens.findtext("image/format") != "R_FLOAT32" or sensor.findtext("topic") != "oak_d_s2/depth":
        raise ValueError("Gazebo depth format or source changed")
    return CameraRig((forward(mount) + forward(sensor), 0., 0.),
                     int(lens.findtext("image/width")), int(lens.findtext("image/height")),
                     float(sensor.findtext("update_rate")), float(lens.findtext("clip/near")),
                     float(lens.findtext("clip/far")))


def simulated_health(*, now_ns, sample_ns, status, flags):
    if sample_ns is None or status is None or flags is None:
        return False
    return (0 <= now_ns - sample_ns <= TRACKING_MAX_AGE_NS and
            0 <= now_ns - status[0] <= TRACKING_MAX_AGE_NS and status[1] and
            0 <= now_ns - flags[0] <= TRACKING_MAX_AGE_NS and flags[1])


def depth_layout(picture, rig, *, float32_enum):
    return (picture.pixel_format_type == float32_enum and picture.width == rig.width and picture.height == rig.height
            and picture.step == 4 * rig.width and len(picture.data) == picture.step * picture.height)


def ros_clock_parts(gazebo_clock):
    value = gazebo_clock.sim
    if not 0 <= value.sec < 2**31 or not 0 <= value.nsec < NS:
        raise ValueError("invalid Gazebo simulation clock")
    return value.sec, value.nsec


def capture_ns(message):
    return _ns(message.header.stamp)
