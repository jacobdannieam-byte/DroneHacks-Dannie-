
# Phase 7 isolated SITL review

Status is **offline rehearsal verified, native integration pending**. This
draft stacks on Phase 6 at `a2c4db5`. It adds a standalone handoff rehearsal and
read-only ROS graph inspection. The rehearsal produces review reasons. The
only production command path remains `refnav.node` through its existing
`Mission` outputs, estimator/depth checks, Offboard supervision and landing.

The handoff rehearsal binds a diagnostic plan to its exact state, alignment,
observation and decision time. It requires a current owner navigation tick,
fresh state and geometry, and the planner's verified clearance and progress.
It rejects expired and future receipts, reference changes, stale or expired
geometry, planner cancellation and timeout, missing proposals, estimator
loss and takeover. A 50 ms handoff-age ceiling is a proposed offline bound.
The owner readiness input in the rehearsal is a fixture flag. A native
design review must identify fresh native sources for each owner gate and
verify semantic validity at the time of use. An eligible result is review
evidence only.

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest -q tests
python -m examples.unified_sitl_rehearsal --output rehearsal.json
# In an isolated ROS 2 Jazzy domain, idle and with the reviewed owner respectively
python -m onboard.diagnostics.sitl_graph
python -m onboard.diagnostics.sitl_graph --expected-owner /refnav
```

The graph tool expects zero publishers on the guarded PX4 command inputs
when idle, or exactly one publisher per input with the expected type and
a unique owner node when active. It reports discovered alternate topic names
with those command types and emits endpoint GIDs for review. A DDS graph
snapshot is point-in-time evidence. Capture repeated observations around
startup, authority transfer and shutdown and corroborate with PX4 telemetry
and the sole XRCE transport.

## Native review order

1. Attach reviewed Phase 1–6 evidence to the exact combined revision. Rerun
   Phase 1 timing with cuVSLAM and PX4 active. Verify Phase 2 native geometry,
   Phase 5 applied-command capture and clock mapping, and the Phase 6
   planner and model budget for the selected configuration.
2. Obtain team approval for the exact isolated SITL run and operator takeover
   path. Start with a fresh world and native landed/disarmed and estimator
   reports. Inspect the isolated ROS domain, graph ownership and calibrated
   sensor streams with the native command owner idle.
3. Review the future proposal-consumption path inside that one owner. Carry
   source/frame/epoch/capture and effective-interval provenance through the
   handshake. Recheck fresh geometry and semantics before consumption. Keep
   watchdog, estimator, depth, authority and landing decisions independent.
   Verify the takeoff and landing path separately before enabling any
   bounded navigation trial.
4. Run the staged sequence: takeoff/hold, mapped waypoint, obstacle avoidance,
   gate/corridor, loss/no route, complete mission and landing. Capture ROS
   bags, PX4 ULogs, graph endpoint evidence, operator/reset records and the
   exact code, settings and course revision for successes and failures.
5. Inject absent/late planner, stale map, VIO loss and operator takeover.
   Record observed watchdog and landing/takeover outcomes. Require independent
   review of the complete flight and failure traces before Phase 8 trials.

The earlier isolated takeoff/hover/land record reports moving RTF about 0.53.
This rehearsal uses synthetic state and owner snapshots. Native Jazzy, PX4
SITL, the course session and target-device data were unavailable locally.
