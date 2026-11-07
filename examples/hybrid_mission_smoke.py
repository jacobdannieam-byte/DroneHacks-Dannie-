"""End-to-end synthetic mission through the actual owner policy and a list sink."""

import argparse
import json
from pathlib import Path

from examples.gazebo_hybrid_adapter import GazeboObservations
from examples.h1_fixture import HEALTH, MS, profile, setup, tick
from examples.hybrid_runtime import HybridRuntime
from refnav.core import Mission, NAV_AUTO_LAND, NAV_OFFBOARD, Params, Vehicle
from refnav.hybrid_mission import CourseMission, Stage
from refnav.trajectory_evaluation import MetricGoal


def runtime_fixture(*, unavailable=False, strategy="hybrid_sitl", **runtime_settings):
    h1 = profile(offset_rig=not unavailable)
    ref = h1.spatial.reference
    goal = lambda x: MetricGoal(ref, (x, .1, 1.3))
    stages = (Stage("start_corridor", "corridor", goal(.40), arrival_m=.14),
              Stage("land", "landing", goal(.85), arrival_m=.14, landing_site_review="fixture_only"))
    from examples.h1_observation import HybridBagReplay

    sources = {name: {"topic": name, "type": kind, "provenance": "synthetic fixture"}
               for name, kind in HybridBagReplay(h1).topics.items()}
    adapter = GazeboObservations(h1, sources, profile_id="fixture", owner_topics={
        "vehicle_attitude": "/fmu/out/vehicle_attitude",
        "vehicle_status": "/fmu/out/vehicle_status_v1",
        "vehicle_land_detected": "/fmu/out/vehicle_land_detected",
    })
    return HybridRuntime(h1, CourseMission(stages, failure_limit=3), adapter=adapter, strategy=strategy,
                         monotonic=runtime_settings.pop("monotonic", lambda: 0.),
                         completion_clock=lambda: 2_000 * MS, **runtime_settings)


class FakeSink:
    """Produces no dynamics or qualification evidence; the scripted state is a fixture."""

    def __init__(self, runtime):
        self.runtime = runtime
        self.profile = runtime.loop.replay.profile
        self.mission = Mission(Params(height_m=.3, climb_rate_mps=4., hover_s=.05,
                                      stream_lead_s=0., health_quiet_s=0., cmd_resend_s=.05),
                               hybrid_navigation=True)
        self.messages = []
        setup(self)

    def feed(self, topic, message, received_ns):
        self.runtime.feed(topic, message, received_ns, received_ns)
        if topic == HEALTH:
            self.mission.on_health(received_ns / 1e9, message.data)

    def step(self, t, x, *, armed, landed, nav_state, z=-1.3, admission=True, owner_ready=True, generation=1,
             with_depth=True):
        tick(self, t, x, with_depth=with_depth)
        now_ns = (t + 3) * MS
        if with_depth:
            self.mission.depth_ready_since = self.mission.depth_t = (t + 2) / 1000
        vehicle = Vehicle(pos=(x, -.10, z), heading=0., pos_ok=True,
                          armed=armed, landed=landed, nav_state=nav_state)
        out = self.runtime.owner_step(self.mission, vehicle, now_ns=now_ns, wall_ns=now_ns,
                                      generation=generation, admission=admission, owner_ready=owner_ready)
        self.messages.append({"tick_ns": now_ns, "phase": out.phase,
                              "commands": out.commands, "setpoint": out.setpoint if out.stream else None})
        return out


def exercise(*, unavailable=False):
    runtime = runtime_fixture(unavailable=unavailable)
    sink = FakeSink(runtime)
    try:
        sink.step(100, 0., armed=False, landed=True, nav_state=0, z=-1.)
        sink.step(150, 0., armed=False, landed=True, nav_state=0, z=-1.)
        sink.step(200, 0., armed=True, landed=True, nav_state=NAV_OFFBOARD, z=-1.)
        for t in (260, 320, 380):
            sink.step(t, 0., armed=True, landed=False, nav_state=NAV_OFFBOARD)
        for t, x in ((520, .1), (660, .35), (800, .40), (940, .51), (1080, .72), (1220, .85), (1360, .85)):
            if sink.mission.phase == "land":
                break
            sink.step(t, 0. if unavailable else x, armed=True, landed=False, nav_state=NAV_OFFBOARD)
        sink.step(1500, 0. if unavailable else .85, armed=True, landed=False, nav_state=NAV_AUTO_LAND)
        sink.step(1640, 0. if unavailable else .85, armed=False, landed=True, nav_state=NAV_AUTO_LAND)
        return {"evidence": "synthetic_scripted_state_fake_sink", "unavailable_route": unavailable,
                "status": runtime.status(), "result": sink.mission.result, "decisions": list(runtime.records),
                "fake_sink": sink.messages}
    finally:
        runtime.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unavailable", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = exercise(unavailable=args.unavailable)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    status = result["status"]
    expected = (status["mission_state"] == "DISARMED"
                and ((result["result"] == "complete") != args.unavailable)
                and any("land" in row["commands"] for row in result["fake_sink"]))
    return int(not expected)


if __name__ == "__main__":
    raise SystemExit(main())
