"""Verify hybrid integration in offline, native shadow, or guarded review mode."""

import argparse
from collections import Counter, deque
import json
from pathlib import Path
import platform
from time import perf_counter_ns

from examples.h1_fixture import MS, profile as fixture_profile, setup, tick
from examples.h1_observation import HybridBagProfile
from examples.hybrid_shadow_loop import ShadowLoop
from examples.spatial_bag_input import read_bag
from examples.unified_planner_ablation import exercise as compare_planners
from refnav.core import NAV_OFFBOARD, Output, Vehicle
from refnav.hybrid_gateway import HybridGateway
from refnav.integration_timing import Timings

REVIEW_GATES = (
    "native_h1_accepted", "continuous_h2_accepted", "h3_gateway_and_fallback_tests_passed",
    "single_owner_and_operator_takeover_reviewed", "exact_isolated_sitl_trial_approved",
)


class ReplayDriver:
    """Adapt the sequential bag reader to rate-limited ticks without sleeping."""

    def __init__(self, loop, report_tick, *, auto_poll, receipt_clock=None):
        self.loop, self.report_tick, self.auto_poll = loop, report_tick, auto_poll
        self.now = None
        self.receipt_clock = receipt_clock

    @property
    def topics(self):
        return self.loop.topics

    @property
    def profile(self):
        return self.loop.replay.profile

    def feed(self, topic, msg, received_ns):
        if self.auto_poll and self.now is not None and received_ns > self.now:
            self.poll(self.now)
        self.now = received_ns
        self.loop.feed(topic, msg, received_ns, received_ns,
                       receipt_wall_ns=None if self.receipt_clock is None else self.receipt_clock())

    def poll(self, now_ns):
        result = self.loop.poll(now_ns, now_ns)
        if result is not None:
            self.report_tick(result)
        if self.loop.fault is not None:
            raise ValueError(self.loop.fault)

    def finish(self):
        self.loop.replay.finish()
        if self.auto_poll:
            self.poll(self.now)


def _fake_owner(loop, tick_result, *, enabled):
    """One local fake sink. Exercise the same setpoint selector as the ROS owner."""
    assessment = tick_result.assessment
    if assessment is None:
        return {"reason": tick_result.reason, "sink_writes": []}
    state = assessment.receipt.state
    baseline = (*state.position_m, state.yaw_rad)
    owner = Output("go", baseline, stream=True)
    vehicle = Vehicle(pos=state.position_m, heading=state.yaw_rad, pos_ok=True,
                      armed=True, landed=False, nav_state=NAV_OFFBOARD)
    gateway = HybridGateway(enabled=enabled, monotonic=loop.monotonic, timings=loop.timings)
    gateway.offer(assessment, owner_generation=1)
    _, inputs = loop.replay.prepare(tick_result.now_ns)
    result = gateway.choose(inputs=inputs, now_ns=tick_result.now_ns, owner_output=owner,
                            vehicle=vehicle, owner_ready=True, owner_generation=1, simulation=True)
    sink = []
    if owner.stream and result.setpoint is not None:
        sink.append(result.setpoint)
    return {"reason": result.reason, "baseline": baseline, "sink_writes": sink}


def offline(args):
    profile = load_profile(args.profile) if args.bag else fixture_profile(offset_rig=args.offset_rig)
    clock = {} if args.enforce_deadline else {"monotonic": lambda: 0.}
    timings = Timings()
    rows = deque(maxlen=2048)
    reasons = Counter()

    def record(result):
        row = result.report()
        row["gateway_disabled"] = _fake_owner(loop, result, enabled=False)
        row["gateway_fake_sitl"] = _fake_owner(loop, result, enabled=True)
        rows.append(row)
        reasons[result.reason] += 1

    skipped = dropped = 0
    for _ in range(args.repeats):
        loop = ShadowLoop(profile, rate_hz=args.rate, timings=timings,
                          measure_receipt_wall=args.enforce_deadline, **clock)
        driver = ReplayDriver(loop, record, auto_poll=args.bag is not None,
                              receipt_clock=perf_counter_ns if args.enforce_deadline else None)
        try:
            if args.bag:
                read_bag(args.bag, driver)
            else:
                setup(driver)
                step = max(200, (loop.period_ns + MS - 1) // MS)
                for i, x in enumerate((0., .3, .6)):
                    time_ms = 100 + i * step
                    tick(driver, time_ms, x)
                    driver.poll((time_ms + 3) * MS)
                driver.finish()
        finally:
            loop.close()
        skipped += loop.skipped_ticks
        dropped += loop.replay.dropped_depth
    return {"evidence": "offline_rosbag" if args.bag else "synthetic_ros_messages",
            "deadline_enforced": args.enforce_deadline, "native_qualification": "incomplete",
            "ticks": list(rows), "reasons": dict(reasons), "timing": timings.summary(),
            "skipped_ticks": skipped, "dropped_depth": dropped}


def load_profile(path):
    return HybridBagProfile.from_dict(json.loads(path.read_text(encoding="utf-8")))


def native(args):
    from examples.hybrid_native_shadow import run

    loop = ShadowLoop(load_profile(args.profile), rate_hz=args.rate, measure_receipt_wall=True)
    rows = deque(maxlen=2048)
    reasons = Counter()

    def record(tick_result):
        rows.append(tick_result.report())
        reasons[tick_result.reason] += 1

    inventory = run(loop, seconds=args.seconds, report_tick=record)
    return {"evidence": "native_read_only_shadow", "native_qualification": "review_required",
            "observations": inventory, "ticks": list(rows), "reasons": dict(reasons),
            "timing": loop.timings.summary(), "skipped_ticks": loop.skipped_ticks,
            "dropped_depth": loop.replay.dropped_depth, "startup_depth_discarded": loop.replay.startup_depth_discarded}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="mode", required=True)
    local = commands.add_parser("offline")
    local.add_argument("--bag", type=Path)
    local.add_argument("--profile", type=Path)
    local.add_argument("--offset-rig", action="store_true")
    local.add_argument("--enforce-deadline", action="store_true")
    local.add_argument("--repeats", type=int, choices=range(1, 101), default=1, metavar="1..100")
    live = commands.add_parser("native-shadow")
    live.add_argument("--profile", type=Path, required=True)
    live.add_argument("--seconds", type=int, choices=range(1, 121), default=30, metavar="1..120")
    comparison = commands.add_parser("compare")
    comparison.add_argument("--repeats", type=int, choices=range(1, 101), default=10, metavar="1..100")
    guarded = commands.add_parser("guarded-review")
    for command in (local, live):
        command.add_argument("--rate", type=int, choices=range(1, 21), default=5, metavar="1..20")
    for command in (local, live, guarded, comparison):
        command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output already exists")
    if args.mode == "offline" and bool(args.bag) != bool(args.profile):
        parser.error("--bag and --profile must be supplied together")
    if args.mode == "offline" and args.bag and (args.repeats != 1 or args.offset_rig):
        parser.error("--repeats and --offset-rig apply only to synthetic fixtures")
    status = 0
    try:
        if args.mode == "guarded-review":
            data = {"evidence": "activation_checklist", "gateway_enabled": False, "requires": REVIEW_GATES,
                    "runbook": "docs/hybrid_shadow_to_sitl.md"}
        elif args.mode == "compare":
            data = compare_planners(args.repeats, None)
        elif args.mode == "offline":
            data = offline(args)
        else:
            data = native(args)
            observations = data["observations"]
            status = int(not any("decision" in row for row in data["ticks"])
                         or observations["fault"] is not None or observations["interrupted"]
                         or any(sample["issues"] for sample in observations["command_graph"]))
    except (OSError, ImportError, ValueError, TypeError, KeyError, AttributeError, RuntimeError) as error:
        data = {"mode": args.mode, "native_qualification": "incomplete", "error": str(error)}
        status = 2
    data["runtime"] = {"platform": platform.platform(), "machine": platform.machine(),
                       "processor": platform.processor(), "python": platform.python_version()}
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(data, output, indent=2, allow_nan=False)
        output.write("\n")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
