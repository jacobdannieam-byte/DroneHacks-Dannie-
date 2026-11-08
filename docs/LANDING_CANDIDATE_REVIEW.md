# Conservative landing candidate and Gazebo acceptance

The prior landing is preserved at tag fallback/landing-before-fix-2026-11-07
(commit cf34dea), and in a verified complete Git bundle in the parent workspace.
That fallback contains the known hazards and is for comparison, not deployment.

## Change

The actual land() entry point now uses the existing 0.35 m/s descent until disarm.
It no longer chooses medium descent or push-down based on height, velocity,
stall or timeout. The old push-down entry point also uses gentle descent.
Landing safety checks and command-authority checks remain active. Already-disarmed
entry still raises ControlIsNoLongerOurs; no final target is sent after disarm.

This contains unsafe command escalation from frozen range and false touchdown.
It does NOT detect all frozen sensors or certify contact. The tradeoff is slower
landing/disarm and reliance on the existing gentle-descent behavior. No new
sensor thresholds, PX4 parameters, publisher or map-control enablement.
Legacy touchdown arithmetic remains for historical tests, but land() does not
use it to authorize commands. No automatic fallback to the old aggressive policy.

## Offline evidence

ROS image drone-simulator:latest, network disabled, read-only source mount:

    source /opt/ros/humble/setup.bash
    PYTHONPATH=/repository/drone_code/team_nav python3 -m pytest drone_code/team_nav/test/test_flight_steps.py drone_code/team_nav/test/test_launch_target.py drone_code/team_nav/test/test_height_check.py drone_code/team_nav/test/test_run_safety.py -q -p no:cacheprovider

70 passed in 2.33s. Initial run: 69 passed/1 failed, because already-disarmed
entry no longer raised; corrected and rerun. An initial attempt to import the
review model in that image failed for missing px4_msgs; no model result claimed.

The reviewed external harness SHA256 is
983d2f19ab80f204bdce02442b820b1b2c69de0a995b97c9de708e87a91705be.
Using drone-simulator-gazebo-hybrid:latest, network disabled, repository and
review folder mounted read-only, source /opt/ros/humble/setup.bash and
/opt/px4_messages/install/setup.bash; set PYTHONPATH to drone_code/team_nav;
run python3 drone_code/team_nav/test/landing_review_replay.py.
The harness expects the supplied review.py at /review/review.py.

All 13 candidate model cases passed: 12 modeled disarms and one authority-loss
exit; no command above 0.35 m/s and no push-down. This includes fresh frozen
range and false-floor/false-zero-velocity. Model disarm is programmed behavior,
not native validation. The original harness and recordings were not changed.

## Updated Gazebo agent plan

1. Preserve any current corridor run and its artifacts. Fetch the candidate,
   record full SHA, image, world, configuration and source-pair hashes. Keep
   map control and background registration disabled for initial landing trials.
2. Reproduce the 70 unit tests and 13 model cases. Compare with the preserved
   fallback. Never select the fallback automatically after a candidate failure.
3. Confirm the actual corrected course_run loads the candidate flight_steps;
   stock route_test alone is not evidence of this path. Verify one owner.
4. Confirm recording has received clock and required topics before starting
   the observation interval. Preserve the recording-startup and H1 freshness
   fixes/limitations independently; this landing change does not address them.
5. Under separate applicable simulation approval, perform a bounded nominal
   climb, hold and descent. Verify gentle command, actual vertical velocity,
   impact/contact behavior, landing duration and independent landed/disarmed
   reports. Do not assume a 0.35 command implies a 0.35 physical contact speed.
6. Inject each review fault through the actual mission input path: stale PX4,
   invalid z, NaN/negative distance, stale/raw invalid/out-of-range data, fresh
   frozen range, false near-floor plus false zero velocity. Include authority
   loss during descent and disarm before the next publication.
7. Pass requires no medium/push command, no publishing after authority loss,
   a complete recording, no collision beyond intended floor contact, and a
   qualified controlled descent/landing. If automatic disarm does not complete,
   report FAIL; use the approved simulator recovery procedure. Do not re-enable
   push-down, change PX4 safety parameters or weaken acceptance thresholds.
8. Repeat the approved first corridor only after descent qualifies. Then compare
   classical versus reasoning-shadow on identical histories. Record whether
   decisions make verified progress and whether actual mission paths invoke
   swept-body checks. Do not silently convert diagnostic evidence into control.
9. Full reasoning-driven SITL requires a separately reviewed consumer-to-owner
   path, bounded planning/registration timing, clearance/stopping qualification
   and fault-response evidence. Keep map control disabled until separately
   authorized and qualified. Hardware remains a later gate.

Return exact commands, hashes, PASS/FAIL/UNVERIFIED per scenario, command and
actual-velocity traces, contact/land/disarm times, ownership graph and logs.
No moving simulator or physical flight was run while creating this candidate.
