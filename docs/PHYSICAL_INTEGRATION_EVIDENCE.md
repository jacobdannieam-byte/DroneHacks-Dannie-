# Physical integration evidence — November 7, 2026 Pacific

Partial P0 delivery; P1 and P2 remain outstanding. No deployment or flight.

## Source and identity

Fetched origin successfully after the restricted-network attempt failed.
`git merge-base HEAD origin/classical` returned
`56f5d3238af813e94363796d623b520c478a22b8`, matching both heads at inspection.
Work began on clean detached classical, then branch
`codex/physical-height-regressions`. The unified checkout was left unchanged.
Neither recording establishes the deployed source revision or confirms that
this is the incident reported by the team.

## Reproduced height mismatch

Read both committed gzip event streams with Python gzip/json, without running
their programs. The course launch record at 37.962 seconds has camera up
-0.002452939050272107 m and PX4 down -0.15490590035915375 m.
The old camera-origin target for a 0.8 m rise is 0.7975470609497279 m.
The preserved organizer adapter, without anchoring, converts this to negative
PX4 down. The implied rise is only 0.6426411605905741 m, outside the existing
0.10 m completion tolerance. The recorded stop at 58.062 seconds names climb
and its 20-second timeout. This reproduces the coordinate mismatch, not vehicle
dynamics or a successful repaired flight.

The existing correction is retained, extracted into a pure calculation so it
can be tested without ROS. Tests call the preserved organizer conversion with
nonzero origins, check repeated calls do not accumulate offsets, and demonstrate
that enabling adapter anchoring would invalidate this correction. The adapter
also clamps targets before conversion; live envelope and anchoring parameters
remain an acceptance gate. No tolerance or timeout changed.

## Separate height and tracking failures

The out-and-back recording stops at 58.147 seconds during
`back: slide to above the launch mark`: PX4 reports no laser height fusion for
one second at 0.40 m above the floor. This is a different stop from climb.
Raw laser availability was false. The event does not prove why fusion stopped
or that pressure was the only remaining source. Preserve the check pending raw
estimator evidence.

Recorded holds cite arrival ages around 0.20–0.29 seconds against the historical
0.20-second limit. Current code uses arrival time, not capture age. Capture-age,
frozen-pose, clock anomaly, arrival-gap and reset analysis remain outstanding.
The current 0.5-second policy has not been qualified by these tests.

## Exact validation

From the parent workspace in PowerShell:

```powershell
$env:PYTHONPATH='classical-audit-source/drone_code/team_nav'
python -m pytest classical-audit-source/drone_code/team_nav/test/test_launch_target.py classical-audit-source/drone_code/team_nav/test/test_height_check.py classical-audit-source/drone_code/team_nav/test/test_run_safety.py -q
```

Result: **37 passed in 0.52s**. The earlier height/safety-only run passed 31.
`python -m pytest classical-audit-source/drone_code/team_nav/test/test_flight_steps.py -q`
initially failed for missing package path; after setting PYTHONPATH it failed
collection because rclpy is absent. The added DroneLink method regressions
remain unexecuted; pure conversion tests passed. Docker is unavailable here.

## Next physical verification for team review

Before any separately approved flight, capture deployed source and configuration
hashes, organizer adapter version, anchor_setpoints, yaw offset and envelope.
With propulsion disabled under the team's procedure, verify the outgoing target
and converted PX4 target for nonzero launch origins, including envelope clamping.
Verify one command owner, fresh capture timestamps and reset counters; collect
laser/estimator evidence. Run the ROS unit suite in its supported environment.
Only after team review should a separately authorized bounded climb and first
horizontal segment be considered, with the existing abort and takeover checks.
No map-based control is enabled by this work.
