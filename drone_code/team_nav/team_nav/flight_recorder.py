"""The flight recorder: a separate program that listens to every ROS topic on the drone and writes
what it hears to disk as it goes, so that we can study a flight afterwards.

REAL DRONE CODE. Start it with "ros2 run team_nav flight_recorder", before the flight program.
drone_code/COLLECTING_RECORDS.md says how to start it and how to copy the records to a laptop.

Why a separate program
----------------------
If the flight program crashes, the record of what led to the crash is what we need most. A
recorder inside the flight program would die with it. This one keeps going, and because it
writes as it goes, even a power cut loses at most about half a second (run_log.py explains how).

It can never move the drone
---------------------------
It has no publishers at all, so it cannot send a target, a command or anything else, and it
never calls a service. rclpy gives every ROS program ("node") one publisher, for announcing
changes to its own settings on /parameter_events; it uses it once while the node starts, and we
then remove it (_remove_every_publisher). The recorder also runs at low priority ("nice"), so
that when the processor is busy, Linux serves the flight program first and the recorder misses
some messages instead.

How it listens, and why that way
--------------------------------
A ROS "topic" is a named stream of messages. The usual way to listen is to give rclpy a function
to call for every message. We measured that this costs too much here: rclpy rechecks every topic
each time any message arrives, and the drone sends well over a thousand messages a second on
about 60 topics. In our simulator that took 40% of one core in competition mode.

So the recorder does something less common, which is faster. Its topics belong to a second node
(the "listening node") that is never handed to rclpy to run. Arriving messages simply wait in
each topic's queue, up to MESSAGES_WAITING_PER_TOPIC of them, and every
SECONDS_BETWEEN_COLLECTIONS the recorder takes all waiting messages out of every queue at once
(_collect_waiting_messages). Each message still carries the time it was sent and the time it
arrived, so nothing is lost by collecting them a little later.

What it records
---------------
Into its own run folder, for example ~/team_ws/flight_records/20261108_101502_competition_flight_recorder/:
  - topics/<topic>.csv: one file per topic, one row per recorded message. Each row has
    seconds_since_start (when the recorder wrote the row, counted from when it started),
    received_unix_time_seconds and sent_unix_time_seconds (when the message arrived and when
    its sender sent it, by the Jetson's clock), then every field of the message, including its
    own time stamps (header.stamp.sec and .nanosec for most ROS messages; "timestamp" in
    microseconds since PX4 started, for PX4's). message_flattening.py explains the columns.
    Every topic it can see is recorded, found by name every few seconds, so topics that appear
    later are recorded too. recorded_topics.py says which are skipped and how often each is
    recorded in competition and in practice mode.
  - computer_health.csv: processor, memory, temperatures, free disk and our programs' use,
    once a second (computer_health.py).
  - events.jsonl: what the recorder decided and noticed: each topic it started recording and
    with what settings, disk space warnings, and every 30 seconds a status line with the rows
    written and its own processor use.
  - session.json: the computer, the ROS settings and a fingerprint of the installed code
    (session_description.py). topics_seen.json: every topic seen, updated as new ones appear.
  - In practice mode only: pictures/<camera topic>/<seconds since start>.png, about one a
    second from each camera, listed in camera_pictures.csv.

How much it costs
-----------------
Measured in our simulator on 2026-11-07, during the landing test, with about 65 topics and about
1,000 messages a second, including pretend cameras at drone 16's rates (simulator/test_flights/
recorder_test.py). Percentages are of one core of an M1 Mac. The Jetson's cores are slower, so
expect roughly two to three times as much there; we have not measured it. The recorder's own
"status" event reports its use every 30 seconds, so the first flight on the drone will tell us.
  - Competition mode: 11.8% on average, 20% at most over 5 seconds; about 425 rows a second;
    about 7 MB of disk a minute; about 76 MB of memory.
  - Practice mode: 26.7% on average, 43% at most over 5 seconds; about 1,060 rows a second
    (27 MB a minute) plus two camera pictures a second (up to 2 MB each, so up to 240 MB a
    minute; real pictures squeeze smaller than our random test pictures); about 99 MB of memory.
  - Before we changed how it listens (see above), competition mode took 40.8%.
  - Killed with SIGKILL mid-flight, every file stayed readable and the record stopped 0.29
    seconds before the kill. A real power cut can also lose what Linux had not yet written to
    the disk; run_log.py forces that out at least every half second. We could not test a power
    cut itself.

What it does not do
-------------------
It does not judge the flight or warn about it, and it does not record camera pictures as rows.
It never deletes records, even when the disk is nearly full: it records less instead
(records_space.py), and if writing fails it carries on without writing (run_log.py).
"""
import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import team_nav
from team_nav.camera_pictures import picture_message_to_png
from team_nav.computer_health import ComputerHealth, own_processor_seconds
from team_nav.message_flattening import flatten_message
from team_nav.recorded_topics import (
    DO_NOT_RECORD, LARGEST_MESSAGE_RECORDED_BYTES, RECORD_FIRST_MESSAGE, SAVE_PICTURES,
    RowRateLimiter, TopicPlan, most_pictures_per_second, most_rows_per_second, picture_topics, plan_for_topic,
    table_name_for_topic, use_reliable_saved_messages,
)
from team_nav.records_space import (
    RECORD_EVERYTHING, describe_free_space, find_records_storage, free_space_bytes, recording_level,
)
from team_nav.run_log import RunLog, records_folder
from team_nav.run_mode import COMPETITION, RunMode, read_run_mode
from team_nav.session_description import describe_session

PROGRAM_NAME = "flight_recorder"
LISTENING_NODE_NAME = "flight_recorder_listener"

# Our own choice. Linux's "niceness" runs from 0 (normal) to 19 (only when nothing else wants
# the processor). 10 lets the flight program and the organizers' programs go first.
RECORDER_NICENESS = 10

# Our own choices for how often the recorder does its regular jobs.
SECONDS_BETWEEN_COLLECTIONS = 0.05
SECONDS_BETWEEN_TOPIC_SEARCHES = 3.0
SECONDS_BETWEEN_HEALTH_ROWS = 1.0
SECONDS_BETWEEN_DISK_CHECKS = 30.0

# Our own choice. How many messages of one topic may wait between two collections. Beyond
# this the oldest are dropped. 50 messages in 0.05 seconds is 1000 a second, more than any
# topic we know of sends. Messages are small (well under a kilobyte), so this costs little memory.
MESSAGES_WAITING_PER_TOPIC = 50

# Only the newest camera picture is kept waiting: each is about 2 MB, and we save one a second.
PICTURES_WAITING_PER_CAMERA = 1

# Our own choice. Messages kept for late listeners, such as /rosout's recent history.
SAVED_MESSAGES_TO_ASK_FOR = 100

NANOSECONDS_PER_SECOND = 1e9


def listening_settings(reliable_and_saved, messages_waiting):
    """The "best effort" and "volatile" settings match every publisher; recorded_topics.py
    explains when we ask for "reliable" and "transient local" instead."""
    if reliable_and_saved:
        return QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=max(messages_waiting, SAVED_MESSAGES_TO_ASK_FOR))
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=messages_waiting)


def remove_every_publisher(node):
    """Removes the publisher rclpy gives every node, and checks that none is left."""
    for publisher in list(node.publishers):
        node.destroy_publisher(publisher)
    if list(node.publishers):
        raise RuntimeError(
            f"The recorder's node {node.get_name()} still has a publisher after removing them all, so it "
            f"could send messages. This should be impossible; do not run it until flight_recorder.py is fixed."
        )


def take_waiting_message(subscription):
    """Takes the oldest waiting message out of a topic's queue, still packed as bytes, without
    unpacking it. Returns (packed bytes, arrival time, sending time), times in Unix seconds, or
    None when the queue is empty. This is the same call rclpy itself makes for each message
    (rclpy/executors.py, _take_subscription)."""
    with subscription.handle:
        taken = subscription.handle.take_message(subscription.msg_type, True)
    if taken is None:
        return None
    packed_message, message_times = taken
    return (packed_message,
            message_times["received_timestamp"] / NANOSECONDS_PER_SECOND,
            message_times["source_timestamp"] / NANOSECONDS_PER_SECOND)


def never_called(_):
    """rclpy needs a function for every subscription. The listening node is never run by rclpy,
    so this is never called; the recorder takes messages itself instead."""


class RecordedTopic:
    """What the recorder knows about one topic, and how much it has recorded."""

    def __init__(self, name, type_name, plan):
        self.name = name
        self.type_name = type_name
        self.plan = plan
        self.table_name = table_name_for_topic(name)
        self.message_class = None
        self.subscription = None
        self.listening_settings = None
        self.rate_limiter = RowRateLimiter()
        self.messages_received = 0
        self.rows_written = 0
        self.has_reported_a_problem = False

    def description(self):
        """For topics_seen.json."""
        return {
            "type": self.type_name,
            "how": self.plan.how,
            "why": self.plan.reason,
            "listening_settings": self.listening_settings,
            "messages_received": self.messages_received,
            "rows_written": self.rows_written,
        }


class FlightRecorder(Node):

    def __init__(self, run_log, run_mode):
        # No log messages of our own on /rosout and no services: both would be ways to talk.
        super().__init__(PROGRAM_NAME, enable_rosout=False, start_parameter_services=False)
        remove_every_publisher(self)
        self.listening_node = Node(LISTENING_NODE_NAME, enable_rosout=False, start_parameter_services=False)
        remove_every_publisher(self.listening_node)
        self.run_log = run_log
        self.run_mode = run_mode
        self.topics = {}                    # Topic name -> RecordedTopic.
        self.recording_level = RECORD_EVERYTHING
        self.picture_topic_names = picture_topics(os.environ)
        self.session_is_saved = False
        self.computer_health = ComputerHealth(run_log.folder)
        self.processor_seconds_at_last_status = own_processor_seconds()
        self.rows_at_last_status = 0
        os.makedirs(os.path.join(run_log.folder, "topics"), exist_ok=True)

        self._check_the_disk()
        self.create_timer(SECONDS_BETWEEN_COLLECTIONS, self._guarded(self._collect_waiting_messages))
        self.create_timer(SECONDS_BETWEEN_TOPIC_SEARCHES, self._guarded(self._search_for_new_topics))
        self.create_timer(SECONDS_BETWEEN_HEALTH_ROWS, self._guarded(self._record_computer_health))
        self.create_timer(SECONDS_BETWEEN_DISK_CHECKS, self._guarded(self._check_the_disk_and_report_status))

    def destroy_node(self):
        self.listening_node.destroy_node()
        super().destroy_node()

    # ---- Finding topics -----------------------------------------------------------------------

    def _search_for_new_topics(self):
        found_a_new_topic = False
        for topic_name, type_names in self.get_topic_names_and_types():
            if topic_name not in self.topics:
                self._start_recording(topic_name, type_names)
                found_a_new_topic = True
        if found_a_new_topic:
            self.run_log.save_text_file("topics_seen.json", json.dumps(self._topics_seen(), indent=1))
        if not self.session_is_saved:
            self._save_session_description()

    def _start_recording(self, topic_name, type_names):
        type_name = type_names[0]
        plan = plan_for_topic(topic_name, type_name, self.run_mode, self.picture_topic_names)
        topic = RecordedTopic(topic_name, type_name, plan)
        self.topics[topic_name] = topic
        if len(type_names) > 1:
            self.run_log.event("topic", f"{topic_name} is sent with several message types {type_names}; "
                                        f"recording only {type_name}.", topic=topic_name)
        if plan.how == DO_NOT_RECORD:
            self.run_log.event("topic", f"Not recording {topic_name}: {plan.reason}.", topic=topic_name, type=type_name)
            return
        if not self._load_message_class(topic):
            return
        reliable_and_saved = use_reliable_saved_messages(self._publisher_settings(topic_name))
        messages_waiting = PICTURES_WAITING_PER_CAMERA if plan.how == SAVE_PICTURES else MESSAGES_WAITING_PER_TOPIC
        topic.listening_settings = "reliable, with saved messages" if reliable_and_saved else "best effort"
        # raw=True keeps each message packed as bytes. Unpacking costs processor time, so we
        # unpack only the messages we record.
        topic.subscription = self.listening_node.create_subscription(
            topic.message_class, topic_name, never_called, listening_settings(reliable_and_saved, messages_waiting),
            raw=True)
        self.run_log.event("topic", f"Recording {topic_name}: {plan.reason}.", topic=topic_name,
                           type=type_name, how=plan.how, listening_settings=topic.listening_settings)

    def _load_message_class(self, topic):
        try:
            topic.message_class = get_message(topic.type_name)
            return True
        except (AttributeError, ModuleNotFoundError, ValueError) as problem:
            topic.plan = TopicPlan(DO_NOT_RECORD, False, "its message type is not installed")
            self.run_log.event("topic", f"Cannot record {topic.name}: its message type {topic.type_name} is "
                                        f"not installed where the recorder runs ({problem}).", topic=topic.name)
            return False

    def _publisher_settings(self, topic_name):
        settings = []
        for publisher in self.get_publishers_info_by_topic(topic_name):
            qos = publisher.qos_profile
            settings.append((qos.reliability == QoSReliabilityPolicy.RELIABLE,
                             qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL))
        return settings

    def _stop_listening(self, topic):
        self.listening_node.destroy_subscription(topic.subscription)
        topic.subscription = None

    # ---- Collecting messages ------------------------------------------------------------------

    def _collect_waiting_messages(self):
        for topic in list(self.topics.values()):
            if topic.subscription is None:
                continue
            try:
                if topic.plan.how == SAVE_PICTURES:
                    self._save_picture_if_due(topic)
                else:
                    self._record_waiting_messages(topic)
            except Exception as problem:  # The recorder must never stop because of one topic.
                self._report_problem_once(topic, problem)

    def _record_waiting_messages(self, topic):
        # At most one queue's worth, so that a very busy topic cannot keep the recorder here.
        for _ in range(MESSAGES_WAITING_PER_TOPIC):
            taken = take_waiting_message(topic.subscription)
            if taken is None:
                return
            packed_message, received_seconds, sent_seconds = taken
            topic.messages_received += 1
            self._record_one_message(topic, packed_message, received_seconds, sent_seconds)
            if topic.plan.how == RECORD_FIRST_MESSAGE and topic.rows_written:
                self._stop_listening(topic)
                return

    def _record_one_message(self, topic, packed_message, received_seconds, sent_seconds):
        limit = most_rows_per_second(topic.plan, self.run_mode, self.recording_level)
        if not topic.rate_limiter.should_keep(received_seconds, limit):
            return
        if len(packed_message) > LARGEST_MESSAGE_RECORDED_BYTES:
            self._report_problem_once(topic, f"a message of {len(packed_message)} bytes is larger than the "
                                             f"{LARGEST_MESSAGE_RECORDED_BYTES} bytes we record")
            return
        message = deserialize_message(packed_message, topic.message_class)
        self.run_log.sample(topic.table_name, received_unix_time_seconds=round(received_seconds, 6),
                            sent_unix_time_seconds=round(sent_seconds, 6), **flatten_message(message))
        topic.rows_written += 1

    def _save_picture_if_due(self, topic):
        if not topic.rate_limiter.should_keep(time.monotonic(), most_pictures_per_second(self.recording_level)):
            return
        taken = take_waiting_message(topic.subscription)
        if taken is None:
            return
        packed_message, received_seconds, _ = taken
        topic.messages_received += 1
        picture = deserialize_message(packed_message, topic.message_class)
        folder = os.path.join(self.run_log.folder, "pictures", topic.table_name.split("/", 1)[1])
        os.makedirs(folder, exist_ok=True)
        file_name = f"{self.run_log.seconds_since_start():09.3f}.png"
        # Not forced onto the disk: a picture lost in a power cut is not worth slowing down for.
        with open(os.path.join(folder, file_name), "wb") as picture_file:
            picture_file.write(picture_message_to_png(picture))
        topic.rows_written += 1
        self.run_log.sample(
            "camera_pictures", received_unix_time_seconds=round(received_seconds, 6), topic=topic.name,
            file=os.path.join("pictures", os.path.basename(folder), file_name),
            stamp_sec=picture.header.stamp.sec, stamp_nanosec=picture.header.stamp.nanosec,
            width=picture.width, height=picture.height, encoding=picture.encoding)

    def _report_problem_once(self, topic, problem):
        if topic.has_reported_a_problem:
            return
        topic.has_reported_a_problem = True
        self.run_log.event("problem", f"Recording {topic.name} had a problem: {problem!r}. The recorder "
                                      f"carries on; later problems with this topic are not reported again.",
                           topic=topic.name)

    # ---- Regular jobs -------------------------------------------------------------------------

    def _guarded(self, job):
        """Wraps a regular job so that an unexpected error is written down instead of stopping
        the recorder."""
        def run_job():
            try:
                job()
            except Exception as problem:
                self.run_log.event("problem", f"The recorder's job {job.__name__} failed: {problem!r}. "
                                              f"It carries on and will try again.")
        return run_job

    def _record_computer_health(self):
        row = self.computer_health.sample()
        row["recorder_processor_seconds_so_far"] = round(own_processor_seconds(), 2)
        self.run_log.sample("computer_health", unix_time_seconds=round(time.time(), 3), **row)

    def _check_the_disk_and_report_status(self):
        self._check_the_disk()
        self._report_status()

    def _check_the_disk(self):
        free_bytes = free_space_bytes(self.run_log.folder)
        new_level = recording_level(free_bytes)
        if new_level != self.recording_level or new_level != RECORD_EVERYTHING:
            self.run_log.event("disk", describe_free_space(free_bytes), free_bytes=free_bytes, recording=new_level)
        self.recording_level = new_level

    def _report_status(self):
        rows_written = self._total_rows_written()
        processor_seconds = own_processor_seconds()
        processor_percent = 100.0 * (processor_seconds - self.processor_seconds_at_last_status) / SECONDS_BETWEEN_DISK_CHECKS
        megabytes_on_disk = _folder_size_bytes(self.run_log.folder) / (1024 * 1024)
        self.run_log.event(
            "status",
            f"Recording {self._listening_topic_count()} topics. Over the last {SECONDS_BETWEEN_DISK_CHECKS:g} "
            f"seconds: {rows_written - self.rows_at_last_status} rows, {processor_percent:.1f}% of one "
            f"processor core. This run's records hold {megabytes_on_disk:.1f} MB.",
            rows_in_period=rows_written - self.rows_at_last_status,
            recorder_processor_percent_of_one_core=round(processor_percent, 1),
            records_megabytes=round(megabytes_on_disk, 2),
            messages_and_rows_per_topic=self._messages_and_rows_per_topic(),
        )
        self.processor_seconds_at_last_status = processor_seconds
        self.rows_at_last_status = rows_written

    def _total_rows_written(self):
        rows_written = 0
        for topic in self.topics.values():
            rows_written += topic.rows_written
        return rows_written

    def _listening_topic_count(self):
        count = 0
        for topic in self.topics.values():
            if topic.subscription is not None:
                count += 1
        return count

    def _messages_and_rows_per_topic(self):
        """{topic: [messages taken so far, rows written so far]}, for topics that sent anything."""
        counts = {}
        for topic in self.topics.values():
            if topic.messages_received:
                counts[topic.name] = [topic.messages_received, topic.rows_written]
        return counts

    def _topics_seen(self):
        seen = {}
        for name in sorted(self.topics):
            seen[name] = self.topics[name].description()
        return seen

    def _save_session_description(self):
        storage = find_records_storage(self.run_log.folder)
        package_folder = os.path.dirname(os.path.abspath(team_nav.__file__))
        topic_types = {}
        for name in sorted(self.topics):
            topic_types[name] = self.topics[name].type_name
        session = describe_session(self.run_mode, topic_types, storage, package_folder)
        self.run_log.save_text_file("session.json", json.dumps(session, indent=1, default=str))
        self.session_is_saved = True

    def final_summary(self):
        """For the last event when the recorder stops."""
        return self._messages_and_rows_per_topic()


def _folder_size_bytes(folder):
    total_bytes = 0
    for current_folder, _, file_names in os.walk(folder):
        for file_name in file_names:
            try:
                total_bytes += os.path.getsize(os.path.join(current_folder, file_name))
            except OSError:
                pass  # A file may vanish while we count; it does not matter here.
    return total_bytes


def _read_run_mode_or_competition():
    """The recorder records even when the mode setting is wrong, because the record matters more
    than the label. It then records as in competition mode, which costs least, and says so."""
    try:
        return read_run_mode(), None
    except ValueError as problem:
        return RunMode(COMPETITION, ""), str(problem)


def _lower_own_priority():
    try:
        os.nice(RECORDER_NICENESS)
    except OSError as problem:
        print(f"The recorder could not lower its own priority ({problem}); it runs at normal priority.", flush=True)


def _start_run_log(run_mode, mode_problem):
    run_log = RunLog(PROGRAM_NAME, run_mode)
    if mode_problem:
        run_log.event("problem", f"The run mode setting is wrong, so the recorder records as in competition "
                                 f"mode: {mode_problem}")
    storage = find_records_storage(run_log.folder)
    if storage is not None:
        run_log.event("storage", storage.explanation, survives_restart=storage.survives_restart,
                      filesystem_type=storage.filesystem_type, mount_point=storage.mount_point)
    return run_log


def main():
    _lower_own_priority()
    run_mode, mode_problem = _read_run_mode_or_competition()
    try:
        run_log = _start_run_log(run_mode, mode_problem)
    except OSError as problem:
        raise SystemExit(f"The recorder cannot create its folder inside {records_folder()} ({problem}). "
                         f"Check that the folder exists and is writable, or set TEAM_NAV_RECORDS_FOLDER.")
    rclpy.init()
    recorder = FlightRecorder(run_log, run_mode)
    run_log.event("start", "The recorder is listening. It has no publishers, so it cannot move the drone.")
    try:
        rclpy.spin(recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C, or the launch system stopping us: both are a normal end.
    finally:
        run_log.event("stop", "The recorder is stopping.", messages_and_rows_per_topic=recorder.final_summary())
        run_log.close()
        recorder.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
