
"""Read normalized shadow evidence from JSONL, rosbag2, or one opt-in ROS subscription."""

import argparse
import json
from pathlib import Path

from refnav.dynamics.adaptive import AdaptiveDynamicsModel, AdaptiveNetwork
from refnav.shadow_replay import ShadowReplay, StreamContract

TOPIC = "/dronehacks/diagnostics/shadow_input"
MESSAGE = "std_msgs/msg/String"
MAX_PAYLOAD = 4096
MAX_BAG_EVENTS = 1_000_000


def _emit(event):
    print(json.dumps(event, allow_nan=False), flush=True)


def consume(replay, payload, *, delivery_ns=None):
    try:
        if len(payload) > MAX_PAYLOAD:
            raise ValueError("payload too large")
        event = json.loads(payload)
        if not isinstance(event, dict):
            raise ValueError("event must be an object")
    except (ValueError, UnicodeError):
        replay.invalidate()
        return {"status": "invalid_payload"}
    return replay.feed(event, delivery_ns=delivery_ns)


def read_bag(path, replay):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from std_msgs.msg import String

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id=""), ConverterOptions("", ""))
    try:
        types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
        if types.get(TOPIC) != MESSAGE:
            raise ValueError(f"bag must contain reviewed {MESSAGE} events on {TOPIC}")
        reader.set_filter(StorageFilter(topics=[TOPIC]))
        count = 0
        while reader.has_next():
            _, data, _ = reader.read_next()
            count += 1
            if count > MAX_BAG_EVENTS or len(data) > MAX_PAYLOAD + 64:
                raise ValueError("bag exceeds event or serialized payload budget")
            _emit(consume(replay, deserialize_message(data, String).data))
    finally:
        reader.close()


def listen(replay, *, use_sim_time):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=[])
    try:
        node = rclpy.create_node("dronehacks_shadow_replay", parameter_overrides=[
            Parameter("use_sim_time", value=use_sim_time),
        ], enable_rosout=False, start_parameter_services=False)
        try:
            def receive(message):
                delivery_ns = node.get_clock().now().nanoseconds - replay.contract.clock_origin_ns
                _emit(consume(replay, message.data, delivery_ns=delivery_ns))

            node.create_subscription(String, TOPIC, receive, QoSProfile(
                depth=4, reliability=ReliabilityPolicy.BEST_EFFORT))
            rclpy.spin(node)
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--checkpoint", type=Path, help="Optional reviewed D2 adaptive checkpoint")
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument("--jsonl", type=Path)
    sources.add_argument("--bag", type=Path)
    sources.add_argument("--enable-shadow", action="store_true")
    parser.add_argument("--use-sim-time", action="store_true")
    args = parser.parse_args(argv)
    if not (args.enable_shadow or args.jsonl or args.bag):
        _emit({"status": "disabled"})
        return
    if args.manifest is None:
        parser.error("an explicit reviewed stream manifest is required")
    if args.use_sim_time and not args.enable_shadow:
        parser.error("--use-sim-time applies to the live subscriber")
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    replay = ShadowReplay(StreamContract(**manifest["stream"]), learned=(
        AdaptiveDynamicsModel(network=AdaptiveNetwork.load(args.checkpoint)) if args.checkpoint else None))
    if args.enable_shadow:
        listen(replay, use_sim_time=args.use_sim_time)
    elif args.bag:
        read_bag(args.bag, replay)
    else:
        with args.jsonl.open(encoding="utf-8") as handle:
            for count in range(MAX_BAG_EVENTS):
                line = handle.readline(MAX_PAYLOAD + 1)
                if not line:
                    break
                if len(line) > MAX_PAYLOAD:
                    raise ValueError("JSONL line exceeds payload budget")
                _emit(consume(replay, line))
            else:
                if handle.read(1):
                    raise ValueError("JSONL exceeds event budget")


if __name__ == "__main__":
    main()
