"""Decides, for each ROS topic the recorder finds, whether to record it and how often.

REAL DRONE CODE. The recorder (flight_recorder.py) asks this file about every new topic, and
asks again about the rate each time a message arrives, because the rate depends on the free
disk space, which can change during a run.

A "topic" is a named stream of messages in ROS, such as /fmu/out/vehicle_status_v1 (PX4's
status reports) or /skybeta/setpoint/pose (the targets our flight program sends).

The rules
---------
  1. Big messages are never recorded as rows: camera pictures, depth pictures and 3D point
     clouds. One camera picture is about 2 MB, so recording them all would cost more processor
     time and disk than everything else together. In practice mode, one picture a second from
     each of drone 16's two cameras is saved as a PNG file instead (camera_pictures.py).
  2. Camera calibration ("camera_info") never changes during a run, but the camera repeats it
     30 times a second, so only its first message is recorded.
  3. A few topics are rare and important, such as PX4's status, its replies to commands, and
     every program's log messages. Every one of their messages is recorded, in every mode.
  4. Every other topic is limited to COMPETITION_ROWS_PER_SECOND rows a second in competition
     mode, and not limited in practice mode. Messages above the limit are skipped, not delayed.
  5. When the disk is nearly full, every limit is lowered (records_space.py says when).

How the limits were chosen is written next to each number, with the measurement behind it.

What it does not do
-------------------
It does not subscribe or write anything. It only answers questions, so it can be tested
without ROS.
"""
from dataclasses import dataclass

from team_nav.records_space import RECORD_A_TRICKLE, RECORD_EVERYTHING, RECORD_LESS

# The four ways a topic can be recorded. Our own names.
RECORD_ROWS = "rows"                 # One CSV row per recorded message.
RECORD_FIRST_MESSAGE = "first message only"
SAVE_PICTURES = "pictures"           # A few PNG pictures per second (practice mode only).
DO_NOT_RECORD = "not recorded"

# Message types that are too big to record as rows. The text says why, for the log.
TOO_BIG_TO_RECORD = {
    "sensor_msgs/msg/Image": "camera pictures are about 2 MB each",
    "sensor_msgs/msg/CompressedImage": "compressed camera pictures are still large",
    "sensor_msgs/msg/PointCloud2": "3D point clouds are large",
    "sensor_msgs/msg/PointCloud": "3D point clouds are large",
    "stereo_msgs/msg/DisparityImage": "depth pictures are large",
    "nav_msgs/msg/OccupancyGrid": "maps are large",
    "theora_image_transport/msg/Packet": "video streams are large",
    "ffmpeg_image_transport_msgs/msg/FFMPEGPacket": "video streams are large",
    # Camera tracking (cuVSLAM) can publish the whole path flown so far, which grows with
    # every message, and drawings meant for display.
    "nav_msgs/msg/Path": "a path grows with every message; the positions are recorded from odometry instead",
    "visualization_msgs/msg/MarkerArray": "drawings for display can be large",
}
CAMERA_PICTURE_TYPE = "sensor_msgs/msg/Image"
CAMERA_CALIBRATION_TYPE = "sensor_msgs/msg/CameraInfo"

# Our own choice. Topics whose name contains one of these are recorded in full, because they
# are rare and each message matters. PX4's names may end in "_v1"; containing the text is enough.
IMPORTANT_TOPIC_NAME_PARTS = (
    "/rosout",                  # Every program's log messages, including the organizers'.
    "vehicle_command",          # Commands sent to PX4, and PX4's replies (vehicle_command_ack).
    "vehicle_status",           # PX4's mode, armed state and pre-flight checks.
    "failsafe_flags",           # Why PX4 would refuse to fly or would take over.
    "vehicle_land_detected",    # Whether PX4 thinks the drone is on the ground.
    "vehicle_control_mode",     # Which of PX4's controllers are switched on.
    "/skybeta/",                # The organizers' summary, and the targets our program sends.
    "/visual_slam/status",      # Whether camera tracking is working.
    "/parameter_events",        # Any program changing a setting.
    "/diagnostics",
)

# Our own choice, checked by measurement: in the simulator, recording every topic at this limit
# used about 12% of one core of the Mac (flight_recorder.py's header has the measurements). 30
# rows a second is the camera tracking's own rate. If the recorder costs too much on the Jetson,
# lowering this is the first thing to try.
COMPETITION_ROWS_PER_SECOND = 30.0

# Our own choices for a nearly full disk (records_space.py says when the disk counts as nearly
# full). The important topics keep a little more than the rest.
ROWS_PER_SECOND_WHEN_DISK_NEARLY_FULL = 5.0
ROWS_PER_SECOND_WHEN_DISK_ALMOST_FULL = 0.5
IMPORTANT_ROWS_PER_SECOND_WHEN_DISK_ALMOST_FULL = 5.0

# Our own choice. Ordinary reports are under 2 KB; this only stops an unexpected huge message
# type that rule 1 does not know about.
LARGEST_MESSAGE_RECORDED_BYTES = 16 * 1024

# Our own choice, for RowRateLimiter: how many messages in a row may be kept close together.
TOKENS_THE_BUCKET_HOLDS = 2.0

# Our own choice for practice mode's camera pictures. One picture a second from each camera
# is enough to see what the drone saw at each moment.
PICTURES_PER_SECOND_PER_CAMERA = 1.0

# The camera topics whose pictures practice mode saves. Drone 16's two cameras, seen on drone 16
# on 2026-11-07 (about 21 pictures a second each, 1024 x 640 pixels, "bgr8"). Every other picture
# topic is skipped, so that extra picture streams cannot add to the processor's load. The
# setting TEAM_NAV_PICTURE_TOPICS, a comma-separated list, replaces this list, for example for
# the simulator's pretend cameras.
# Switched off on 2026-11-08: every program that reads the 2 MB pictures makes camera tracking's
# reports late (team_nav/transport/pictures_over_network.xml), and saved pictures are not needed
# to fly. To save them again, put back ("/oak/left/image_raw", "/oak/right/image_raw").
DRONE_16_CAMERA_TOPICS = ()
PICTURE_TOPICS_SETTING = "TEAM_NAV_PICTURE_TOPICS"


@dataclass(frozen=True)
class TopicPlan:
    how: str            # One of RECORD_ROWS, RECORD_FIRST_MESSAGE, SAVE_PICTURES, DO_NOT_RECORD.
    is_important: bool  # True if rule 3 applies.
    reason: str         # One sentence for the log.


def plan_for_topic(topic_name, type_name, run_mode, picture_topic_names):
    """How to record one topic. picture_topic_names lists the camera topics whose pictures
    practice mode saves (see picture_topics)."""
    if type_name == CAMERA_PICTURE_TYPE and run_mode.is_practice and topic_name in picture_topic_names:
        return TopicPlan(SAVE_PICTURES, False,
                         f"practice mode saves {PICTURES_PER_SECOND_PER_CAMERA:g} picture(s) a second as PNG files")
    if type_name in TOO_BIG_TO_RECORD:
        return TopicPlan(DO_NOT_RECORD, False, TOO_BIG_TO_RECORD[type_name])
    if type_name == CAMERA_CALIBRATION_TYPE:
        return TopicPlan(RECORD_FIRST_MESSAGE, False, "the calibration does not change during a run")
    if is_important_topic(topic_name):
        return TopicPlan(RECORD_ROWS, True, "every message is recorded, because each one matters")
    if run_mode.is_practice:
        return TopicPlan(RECORD_ROWS, False, "practice mode records every message")
    return TopicPlan(RECORD_ROWS, False,
                     f"competition mode records at most {COMPETITION_ROWS_PER_SECOND:g} rows a second")


def picture_topics(environment):
    """The camera topics whose pictures practice mode saves."""
    setting = environment.get(PICTURE_TOPICS_SETTING, "").strip()
    if not setting:
        return DRONE_16_CAMERA_TOPICS
    names = []
    for name in setting.split(","):
        if name.strip():
            names.append(name.strip())
    return tuple(names)


def is_important_topic(topic_name):
    for name_part in IMPORTANT_TOPIC_NAME_PARTS:
        if name_part in topic_name:
            return True
    return False


def most_rows_per_second(plan, run_mode, recording_level):
    """The limit for one topic right now, or None for no limit."""
    if recording_level == RECORD_A_TRICKLE:
        if plan.is_important:
            return IMPORTANT_ROWS_PER_SECOND_WHEN_DISK_ALMOST_FULL
        return ROWS_PER_SECOND_WHEN_DISK_ALMOST_FULL
    if plan.is_important:
        return None
    if recording_level == RECORD_LESS:
        return ROWS_PER_SECOND_WHEN_DISK_NEARLY_FULL
    if run_mode.is_practice:
        return None
    return COMPETITION_ROWS_PER_SECOND


def most_pictures_per_second(recording_level):
    """Pictures stop altogether as soon as the disk is nearly full."""
    if recording_level == RECORD_EVERYTHING:
        return PICTURES_PER_SECOND_PER_CAMERA
    return 0.0


def table_name_for_topic(topic_name):
    """The CSV file name for a topic, inside the "topics" folder: /fmu/out/vehicle_status_v1
    becomes topics/fmu__out__vehicle_status_v1. ROS forbids "__" inside topic names, so two
    topics can never get the same file."""
    return "topics/" + topic_name.strip("/").replace("/", "__")


def use_reliable_saved_messages(publisher_settings):
    """Whether to ask for "reliable" delivery and for messages sent before we started listening
    ("transient local"). publisher_settings has one (is_reliable, keeps_messages_for_late_listeners)
    pair per program publishing the topic.

    Normally we ask for neither ("best effort" and "volatile"), because those settings match
    every publisher, whatever it offers. But some topics are sent once and kept for late
    listeners, such as /tf_static (where each camera is fixed on the drone) and /rosout's recent
    history. We would miss those with the normal settings. So if every publisher of a topic
    offers both, we ask for both. Asking for both from a publisher that does not offer them
    would mean receiving nothing, which is why we check every publisher first."""
    if not publisher_settings:
        return False
    for is_reliable, keeps_messages_for_late_listeners in publisher_settings:
        if not (is_reliable and keeps_messages_for_late_listeners):
            return False
    return True


class RowRateLimiter:
    """Keeps at most most_per_second messages a second, on average.

    It works like a bucket of tokens that refills at most_per_second tokens a second and holds
    at most TOKENS_THE_BUCKET_HOLDS. Each kept message uses one token. So a topic a little slower
    than the limit is still recorded in full even when its messages arrive unevenly, and a much
    faster topic is thinned out evenly. (A simpler rule, "keep a message only if 1/30 s has
    passed since the last one", lost over a third of a 27-a-second topic in the simulator,
    because some of its messages arrived a few milliseconds early.)"""

    def __init__(self):
        self.tokens = TOKENS_THE_BUCKET_HOLDS
        self.last_seconds = None

    def should_keep(self, now_seconds, most_per_second):
        if most_per_second is None:
            return True
        if most_per_second <= 0:
            return False
        # The times may come from the Jetson's clock, which can be set backwards (for example
        # when it learns the right time). Then we just start counting again from the new time.
        if self.last_seconds is not None and now_seconds > self.last_seconds:
            refill = (now_seconds - self.last_seconds) * most_per_second
            self.tokens = min(TOKENS_THE_BUCKET_HOLDS, self.tokens + refill)
        self.last_seconds = now_seconds
        if self.tokens < 1.0:
            return False
        self.tokens -= 1.0
        return True
