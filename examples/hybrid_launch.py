"""Configure classical navigation, subscriber-only shadow, or a separately approved SITL owner."""

import argparse
import json
import os
from pathlib import Path
import sys
from time import monotonic, perf_counter_ns

from examples.gazebo_hybrid_adapter import GazeboObservations, IsolationApproval
from examples.h1_observation import HybridBagProfile
from examples.hybrid_runtime import HybridRuntime, STRATEGIES
from refnav.hybrid_mission import CourseMission, Stage
from refnav.reasoning.schema import MissionGoal
from refnav.trajectory_evaluation import MetricGoal


def _read(path):
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def configured(path):
    doc = _read(path)
    strategy = doc.get("strategy", "classical")
    if strategy not in STRATEGIES:
        raise ValueError("unknown runtime strategy")
    if strategy == "classical":
        return strategy, None
    if doc.get("review_status") != "reviewed":
        raise ValueError("simulator profile incomplete: review native observations, geometry and source evidence")
    root = Path(path).parent
    profile = HybridBagProfile.from_dict(_read(root / doc["h1_profile"]))
    mission = _read(root / doc["mission_profile"])
    stages = []
    ref = profile.spatial.reference
    if (mission.get("reference") != {"frame_id": ref.frame_id, "epoch": ref.epoch, "clock_id": ref.clock_id}
            or not isinstance(mission.get("route_review"), str) or not mission["route_review"].strip()
            or not isinstance(mission.get("stages"), list) or not 1 <= len(mission["stages"]) <= 32):
        raise ValueError("mission needs a bounded reviewed route on the H1 frame, clock and epoch")
    for raw in mission["stages"]:
        fields = dict(raw)
        objective = MetricGoal(ref, fields.pop("position_m"))
        alternatives = tuple(MetricGoal(ref, point) for point in fields.pop("alternatives_m", ()))
        semantic = fields.pop("semantic", None)
        stages.append(Stage(goal=objective, alternatives=alternatives,
                            semantic=MissionGoal.from_dict(semantic) if semantic is not None else None, **fields))
    course = CourseMission(stages, **mission.get("recovery", {}))
    adapter = GazeboObservations(profile, doc["sources"], profile_id=doc["profile_id"],
                                 owner_topics=doc.get("owner_topics"))
    settings = doc.get("runtime", {})
    if set(settings) - {"plan_hz", "semantic_hz", "budget_s", "lease_s"}:
        raise ValueError("unexpected runtime settings")
    logging = doc.get("logging", {})
    if set(logging) - {"record_limit"}:
        raise ValueError("unexpected logging settings")
    if strategy == "hybrid_sitl" and (not adapter.can_attach_owner or not stages[-1].landing_site_review):
        raise ValueError("SITL requires a reviewed owner graph and landing site")
    runtime = HybridRuntime(profile, course, strategy=strategy, adapter=adapter, **settings, **logging)
    if 'stopping_envelope' in doc:
        import hashlib
        from refnav.stopping_envelope import StoppingEnvelope
        envelope = dict(doc['stopping_envelope'])
        measurement = root / envelope.pop('measurement_file')
        if hashlib.sha256(measurement.read_bytes()).hexdigest() != envelope['evidence_sha256']:
            raise ValueError('stopping measurement identity mismatch')
        from refnav.stopping_measurement import validate_measurement
        measured_envelope = StoppingEnvelope(**envelope)
        validate_measurement(_read(measurement), measured_envelope)
        runtime.executor.stopping_envelope = measured_envelope
    return strategy, runtime


def run_reviewed_sitl(runtime, approval, *, ros_args, output):
    """Deployment-owned entry point; unavailable from CLI or a profile flag alone."""
    if (runtime.strategy != "hybrid_sitl" or not isinstance(approval, IsolationApproval)
            or not approval.permits(runtime.adapter, os.environ.get("ROS_DOMAIN_ID"))):
        raise ValueError("live isolated-SITL approval refused")
    if runtime.executor.stopping_envelope is None:
        raise ValueError('reviewed measured stopping envelope required before native policy execution')
    output = Path(output)
    if output.exists():
        raise ValueError("choose a new runtime log output")
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.utilities import try_shutdown
    from onboard.diagnostics.sitl_graph import inspect as inspect_command_graph
    from refnav.node import RefNav

    rclpy.init(args=list(ros_args))
    node = None
    executor = SingleThreadedExecutor()
    code = 1
    try:
        probe = rclpy.create_node("hybrid_preflight_graph", enable_rosout=False,
                                  start_parameter_services=False)
        try:
            until = monotonic() + 4.
            while rclpy.ok() and monotonic() < until:
                rclpy.spin_once(probe, timeout_sec=.1)
            if not rclpy.ok() or inspect_command_graph(probe, strict_motion=runtime.adapter.simulated_estimator)["issues"]:
                raise ValueError("command graph is not idle")
        finally:
            probe.destroy_node()
        node = RefNav(hybrid_runtime=runtime, hybrid_approval=approval)
        executor.add_node(node)
        executor.spin()
    except SystemExit as stop:
        code = int(stop.code or 0)
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    finally:
        runtime.close()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        try_shutdown()
        with output.open("x", encoding="utf-8") as log:
            json.dump({"strategy": runtime.strategy, "records": list(runtime.records),
                       "status": runtime.status(), "timing": runtime.timings.summary(),
                       "total_records": runtime.record_count,
                       "clock_ingestion": runtime.clock_fence.report() if runtime.clock_fence else None,
                       "last_issue": runtime.last_issued, "exit_code": code}, log, indent=2, allow_nan=False)
            log.write("\n")
    return code


def attach_observations(node, runtime):
    """Subscribe on the owner executor using the same bounded settings as H2 shadow."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    static = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    clock_seen = False

    def receive(topic):
        def callback(message):
            nonlocal clock_seen
            now = node.get_clock().now().nanoseconds
            if topic != "/tf_static" and (now <= 0 or (not clock_seen and topic != "/clock")):
                return
            if topic == "/clock":
                clock_seen = True
            runtime.feed(topic, message, now, perf_counter_ns())
        return callback

    for topic, kind in runtime.topics.items():
        node.create_subscription(get_message(kind), topic, receive(topic), static if topic == "/tf_static" else sensor)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--check", action="store_true", help="validate configuration without starting ROS")
    parser.add_argument("--seconds", type=int, default=30, choices=range(1, 121), metavar="1..120")
    parser.add_argument("--output", type=Path)
    args, ros_args = parser.parse_known_args(argv)
    try:
        if ros_args and ros_args[0] != "--ros-args":
            raise ValueError("unrecognized arguments (only ROS arguments can follow)")
        strategy, runtime = configured(args.config) if args.config else ("classical", None)
        if args.check:
            print(json.dumps({"strategy": strategy, "config_valid": True, "command_authority": False}))
            return 0
        if strategy == "hybrid_sitl":
            raise ValueError("no Gazebo activation launcher is approved; use the reviewed deployment adapter")
        if strategy == "classical":
            from refnav.launch import arguments
            from refnav.node import main as classical
            classical(arguments(Path(__file__).resolve().parents[1], height_m=1.2, hover_s=5., ros_args=ros_args))
        if ros_args or args.output is None or args.output.exists():
            raise ValueError("shadow needs a new --output file; ROS domain comes from the environment")
        from examples.hybrid_native_shadow import run

        evidence = run(runtime, seconds=args.seconds, report_tick=lambda _: None)
        with args.output.open("x", encoding="utf-8") as log:
            json.dump({"strategy": strategy, "native_qualification": "review_required", "observations": evidence,
                       "records": list(runtime.records), "timing": runtime.timings.summary(),
                       "status": runtime.status(), "total_records": runtime.record_count}, log, indent=2, allow_nan=False)
            log.write("\n")
        return int(evidence["fault"] is not None or evidence["interrupted"] or not any(
            row.get("decision") for row in runtime.records) or any(r["issues"] for r in evidence["command_graph"]))
    except (OSError, ImportError, ValueError, TypeError, KeyError, RuntimeError) as error:
        print(f"hybrid_launch: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
