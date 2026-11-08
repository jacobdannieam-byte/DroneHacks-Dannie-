"""Native subscriber-only H2 runner. Imports ROS only when invoked."""

from collections import Counter, defaultdict, deque
import os
from shutil import which
from subprocess import SubprocessError, run as run_command
from time import perf_counter_ns, process_time_ns, time_ns
from threading import Event, Thread

from examples.h1_observation import TF
from onboard.diagnostics.sitl_graph import inspect


def transport_metadata(info, *, callback_system_ns):
    """Preserve optional RMW evidence; Humble may provide only the message."""
    def field(name):
        return info.get(name) if isinstance(info, dict) else getattr(info, name, None)
    return {"callback_system_ns": callback_system_ns,
            "message_info": "unavailable" if info is None else "provided",
            "rmw_received_ns": field("received_timestamp"),
            "rmw_source_ns": field("source_timestamp"),
            "publication_sequence": field("publication_sequence_number")}


def optional_message_info(callback):
    """Accept both one-argument Humble delivery and two-argument RMW delivery."""
    def observe(message, info=None):
        return callback(message, info)
    return observe


class ClockExecutorService:
    """Service only a ROS TimeSource; never touch spatial or mission state.

    The clock comes from delivered /clock callbacks, never a sensor's source
    stamp. Keeping its executor independent prevents slow spatial callbacks from
    freezing the receipt clock used to timestamp the next sensor callback.
    """
    def __init__(self, executor, clock, *, okay=lambda: True):
        self.executor, self.clock, self.okay = executor, clock, okay
        self.stop = Event()
        self.failure = None
        self.thread = Thread(target=self._spin, name='shadow-clock-reception', daemon=True)

    def _spin(self):
        try:
            while not self.stop.is_set() and self.okay():
                self.executor.spin_once(timeout_sec=.01)
        except Exception as error:
            if not self.stop.is_set():
                self.failure = {'type': type(error).__name__, 'reason': str(error)}

    def start(self):
        self.thread.start()

    def now_ns(self):
        if self.failure is not None:
            raise ValueError('native_clock_service_failed:' + self.failure['reason'])
        return self.clock.now().nanoseconds

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.)
        if self.thread.is_alive():
            raise RuntimeError('native_clock_service_shutdown_timeout')


def _gpu_snapshot():
    executable = which("nvidia-smi")
    if executable is None:
        return {"status": "unavailable"}
    try:
        result = run_command([executable, "--query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu",
                              "--format=csv"], check=True, capture_output=True, text=True, timeout=2)
        return {"status": "single_post_window_sample", "csv": result.stdout.strip()}
    except (OSError, SubprocessError):
        return {"status": "unavailable"}


def run(loop, *, seconds, report_tick):
    import resource

    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    try:
        from rclpy.event_handler import SubscriptionEventCallbacks, UnsupportedEventTypeError
    except ModuleNotFoundError as error:
        if error.name != "rclpy.event_handler":
            raise
        from rclpy.qos_event import SubscriptionEventCallbacks, UnsupportedEventTypeError
    from rclpy.parameter import Parameter
    from rclpy.utilities import try_shutdown
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    counts, lost = Counter(), Counter()
    transport = defaultdict(lambda: deque(maxlen=64))
    loss_unavailable = []
    clock_seen = False
    rclpy.init(args=[])
    node = None
    clock_node = clock_executor = clock_service = None
    start = perf_counter_ns()
    cpu_start = process_time_ns()
    try:
        clock_node = rclpy.create_node("dronehacks_shadow_receipt_clock", enable_rosout=False,
                                 start_parameter_services=False,
                                 parameter_overrides=[Parameter("use_sim_time", value=True)])
        clock_executor = SingleThreadedExecutor()
        clock_executor.add_node(clock_node)
        clock_service = ClockExecutorService(clock_executor, clock_node.get_clock(), okay=rclpy.ok)
        clock_service.start()
        # This node's wall clock is not used for receipt stamps. The independent
        # TimeSource above receives /clock while this node processes depth.
        node = rclpy.create_node("dronehacks_hybrid_shadow", enable_rosout=False,
                                 start_parameter_services=False)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        static = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def observe(topic):
            def callback(message, info):
                nonlocal clock_seen
                wall = perf_counter_ns()
                now = clock_service.now_ns()
                record = transport_metadata(info, callback_system_ns=time_ns())
                record.update(callback_ros_ns=now, callback_monotonic_ns=wall)
                transport[topic].append(record)
                counts[topic] += 1
                if topic != TF and (now <= 0 or (not clock_seen and topic != "/clock")):
                    counts["startup_discarded"] += 1
                    return
                if topic == "/clock":
                    clock_seen = True
                try:
                    loop.feed(topic, message, now, wall)
                finally:
                    record['feed_finished_monotonic_ns'] = perf_counter_ns()
                    record['feed_duration_ns'] = record['feed_finished_monotonic_ns'] - wall
            return optional_message_info(callback)

        def on_lost(topic):
            def callback(event):
                lost[topic] += event.total_count_change
            return callback

        for topic, kind in loop.topics.items():
            profile = static if topic == TF else sensor
            msg_type = get_message(kind)
            try:
                node.create_subscription(msg_type, topic, observe(topic), profile,
                                         event_callbacks=SubscriptionEventCallbacks(message_lost=on_lost(topic)))
            except UnsupportedEventTypeError:
                loss_unavailable.append(topic)
                node.create_subscription(msg_type, topic, observe(topic), profile,
                                         event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))

        first_clock = last_clock = None
        last_graph = 0
        graphs = []
        interrupted = False
        observation_error = None
        try:
            while rclpy.ok() and perf_counter_ns() - start < seconds * 1e9:
                rclpy.spin_once(node, timeout_sec=.01)
                wall = perf_counter_ns()
                now = clock_service.now_ns()
                if not clock_seen:
                    continue
                if first_clock is None:
                    first_clock = (now, wall)
                last_clock = (now, wall)
                if wall - last_graph >= 1_000_000_000:
                    strict = bool(getattr(getattr(loop, "adapter", None), "simulated_estimator", False))
                    graphs.append(inspect(node, strict_motion=strict))
                    last_graph = wall
                tick = loop.poll(now, wall)
                if tick is not None:
                    report_tick(tick)
                if loop.fault is not None:
                    break
        except (KeyboardInterrupt, ExternalShutdownException):
            interrupted = True
        except ValueError as error:
            # Preserve a measured input rejection instead of losing the entire
            # native report. Never resume this observation epoch after a fault.
            observation_error = {'type': type(error).__name__, 'reason': str(error)}
        finally:
            loop.close()

        elapsed = perf_counter_ns() - start
        offered = {}
        for topic in loop.topics:
            offered[topic] = [
                {"node": f"{info.node_namespace.rstrip('/')}/{info.node_name}", "type": info.topic_type,
                 "reliability": info.qos_profile.reliability.name, "durability": info.qos_profile.durability.name,
                 "depth": info.qos_profile.depth, "history": info.qos_profile.history.name}
                for info in node.get_publishers_info_by_topic(topic)
            ]
        rtf = None
        if first_clock is not None and last_clock[1] > first_clock[1]:
            rtf = (last_clock[0] - first_clock[0]) / (last_clock[1] - first_clock[1])
        return {"topic_types": dict(node.get_topic_names_and_types()), "offered_qos": offered,
                "requested_qos": {"sensor": "best_effort/volatile/keep_last=1",
                                  "tf_static": "reliable/transient_local/keep_last=16"},
                "command_graph": graphs, "received_callbacks": dict(counts), "dds_reported_loss": dict(lost),
                "loss_event_unavailable": loss_unavailable, "observed_rtf": rtf,
                "transport_clock_evidence": {topic: list(rows) for topic, rows in transport.items()},
                "process_cpu_percent": 100 * (process_time_ns() - cpu_start) / elapsed,
                "peak_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                "gpu": _gpu_snapshot(), "ros_distro": os.environ.get("ROS_DISTRO"),
                "rmw": os.environ.get("RMW_IMPLEMENTATION"), "interrupted": interrupted,
                "fault": loop.fault or (observation_error['reason'] if observation_error else None),
                "observation_error": observation_error,
                "receipt_clock_service": {"mode": "independent_ros_timesource_executor",
                                          "failure": clock_service.failure},
                "clock_ingestion": loop.clock_fence.report() if getattr(loop, 'clock_fence', None) else None,
                "clock_observed": clock_seen}
    finally:
        loop.close()
        if clock_service is not None:
            clock_service.close()
        if clock_executor is not None:
            clock_executor.shutdown()
        if clock_node is not None:
            clock_node.destroy_node()
        if node is not None:
            node.destroy_node()
        try_shutdown()
