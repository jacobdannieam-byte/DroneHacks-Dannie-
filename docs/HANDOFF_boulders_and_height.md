# Handoff: boulders and PX4's height

First written 2026-11-07 when this work was paused; updated the evening of 2026-11-07, after the
safety checks were extended, the surprise-boulder flights were flown with the real program, the
referee was fixed and the window's line-up was moved. It says what is done, what the flights
showed, what is still open, and what the team has to decide. Background is in
[`ARCHITECTURE.md`](ARCHITECTURE.md), sections 4 (the route), 7 (the laser and PX4's height) and
11 (safety checks).

Everything here was tested in the simulator only. Nothing was changed on the real drone.

## The problem in two sentences

When the bottom laser passes over a boulder or the window's lower wall, its reading jumps. PX4,
the flight controller's software, may then believe the drone's height changed, or stop trusting
the laser, and its height estimate can end up tens of centimetres wrong.

## What changed this evening

**1. The height check now runs in every stage where the drone is in the air.** Before, the real
program (`course_run.py`) ran its safety checks during the climb, the route and the settle above
the pad, but not during the landing's descent, and not while the drone held still waiting for
camera tracking. Now:

- The descent runs the checks every cycle until touchdown (`flight_steps.DescentHeight`). If PX4's
  height goes wrong during the descent and the laser's raw reading exists, the rest of the
  descent finds the floor with the raw reading. On the real drone, which has no raw reading,
  there is nothing better to switch to, so it is only written down. The checks stop at
  touchdown on purpose, because PX4's height drifts while the drone presses down on the floor.
- The height check also runs while the drone holds still for camera tracking (`run_safety.py`).
- **A new, third form of the height check compares PX4's height with camera tracking's height**
  (`height_check.CameraHeightCheck`), anywhere on the course, with a loose 30 cm limit for 1
  second. This was needed because of the first flight below: PX4's height went 50 cm wrong
  while PX4 still reported that it was using the laser, so the check the real drone relies on
  (`LaserUseCheck`, which watches PX4's own "using the laser" flag) would never have fired. In
  normal flights the two heights stayed within 10 cm, so 30 cm leaves 20 cm for camera
  tracking's height drift, which nobody has measured on the real drone. The route follower
  already needs camera height within 10 cm at every corner, so a much larger drift would stall
  the run anyway.

`flight_steps.py`'s public functions are unchanged for existing callers: `land` and the two
descent steps only gained an optional last argument.

**2. The referee no longer reports two harmless things as "PX4's height went wrong"**
(`boulder_watch.py`, used by `referee.py`):

- The time on the floor after touchdown is not counted (it is still reported, for information).
- The takeoff climb straight above the launch mark has its own 20 cm limit. The blip there is
  real: PX4's height trails the true height by up to 12 cm while climbing, so the drone
  overshoots its cruise height by about 7 cm, and the gap is gone 2 to 3 s after the climb. It
  is harmless there, because the drone is over the launch mark with a metre of air above it and
  only starts the route after holding still for 2 s. The same trailing shows up in any climb,
  including the climb to the window's height, so it matters there (see "What is left").
- The rest of the flight keeps the 10 cm limit.

**3. The window's line-up spot moved away from the boulder** (`opening_crossings.py`). The drone
used to climb to the window's height while flying towards the wall, so it lined up 1.2 m in
front of the wall, where the safety net pinned the spot 13 cm from the boulder in front of the
window. Now it lines up 0.56 m from the wall, stops, climbs straight up on the spot, stops again,
and flies level through. On our scan the line-up is now 47 cm from that boulder's edge, both
window stops are at least 40 cm from every boulder in our scan and in the organizers' drawing,
the hairpin in front of the window is gone, and the window route is 0.8 m shorter (18.3 m instead
of 19.1 m). A new test checks the window stops on all three of our maps; it fails with the old
planner.

All 239 unit tests pass (command below).

```bash
docker run --rm --network none --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 -m pytest drone_code/team_nav/test -q -p no:cacheprovider
```

## The flights

All with the real program (`course_run`, through `simulator/test_flights/real_program_test.py`),
planning on our scan map. Flights 1 to 5 and 7 are in worlds where boulders are not where the
map says; flights 6 and 8 are in our scan's own world. "No raw laser"
means `--ignore-raw-laser`, which makes the program behave as on the real drone. Times are
seconds after takeoff. Height errors are the referee's, in the air.

| # | World and settings | What happened | Result |
|---|---|---|---|
| 1 | Side gap. A boulder added in the first corridor. Drone 16's settings, raw laser. | The drone crossed the boulder at 0.4 to 0.5 m/s. PX4 at once took the boulder's top for a 30 cm drop in height, then stopped following the laser, while its flag still said "using the laser". It climbed to 1.0 m believing it was at 0.5 m. The raw-laser check stopped the run 2 s later, still in the corridor, and the drone landed by the raw laser. | Stopped and landed safely, no touch, 21 s. |
| 2 | Side gap. Boulder 2 moved under the corner after the tunnel. Drone 16's settings, no raw laser. | The drone stopped at the corner right above the boulder. PX4 stopped using the laser; the check fired after 1 s and the drone landed where it was. | Stopped, but **landed on the boulder** (one touch). Height error 23 cm during the landing. |
| 3 | Side gap. The drawing's boulders removed. Drone 16's settings, raw laser. | Normal flight. The descent's checks ran without a false alarm. | **Clean finish** on the pad, no touch, motors off at 100 s, height error 9 cm. |
| 4 | Window. Boulder 3 moved under the old line-up spot. Drone 16's settings, no raw laser. | The drone stopped above the boulder to line up (3.1 s). PX4 stopped using the laser; the check fired and the drone landed where it was. During the landing the new camera check also fired (PX4 37 cm below camera tracking). | Stopped, but **landed against the boulder** (one touch). |
| 5 | As 4, with `EKF2_RNG_K_GATE` = 3, raw laser. | The gate did not help above the boulder: PX4 stopped using the laser at once. The window crossing itself was fine (error under 3 cm). On the way to the pad PX4's height drifted to 17 cm; the raw-laser check stopped the run and the drone landed 0.85 m from the pad's centre. Separately, after the tunnel a propeller brushed the C1 chicane wall, because camera tracking had drifted 12 cm sideways. | Stopped and landed safely, but one touch on the way (the wall brush, unrelated to boulders). |
| 6 | Window, new line-up, our scan world (boulders where the map says), `EKF2_RNG_K_GATE` = 3, raw laser. | The new line-up worked: the drone climbed on the spot 47 cm from the boulder and went through the window without touching, though 8 cm above the window's middle (6 cm to the frame). Over the window's lower wall, PX4 took the wall's top for the floor and kept believing it after the wall, with its flag still on. During the descent after the window its height stayed near 1.0 m while the drone really sank to 0.37 m. The new camera check stopped the run (PX4 1.01 m, camera 0.41 m) and the drone landed by the raw laser. | Stopped and landed safely, no touch, motors off at 95 s, 0.85 m from the pad's centre (off the pad). Height error 78 cm. |
| 7 | As 1, with `EKF2_RNG_K_GATE` = 3, no raw laser. | Same as flight 1: PX4 took the boulder's top for a 30 cm drop, with its flag still on, and the drone climbed to 1.24 m. The gate made no difference. The new camera check stopped the run about 1.5 s after the drop. Watching the flag alone would not have. | Stopped and landed safely, no touch, motors off at 20 s. Height error 50 cm. |
| 8 | As 6, repeated. | Normal flight. Crossed the window at its middle (11.5 cm to the frame). | **Clean finish** on the pad, no touch, motors off at 110 s. Height error 11 cm, 6 s after the window, just over the referee's 10 cm. |

Flights 4 and 5 used the old planner (the world was made to put a boulder under the old
line-up spot); the new planner no longer stops there.

### What the flights taught us

- **The safety checks now cover every airborne stage, and they fired every time PX4's height
  went wrong.** In no flight did the drone carry on with a badly wrong height.
- **A boulder crossed at full speed can also upset PX4** (flight 1), not only a boulder under a
  stop. Its height followed the boulder's top by 30 cm within half a second. On drone 16's
  settings, a surprise boulder anywhere on the route can end the run.
- **PX4's "using the laser" flag is not enough on its own** (flights 1, 6 and 7). That is why the
  camera height comparison was added, and in flights 6 and 7 it was the check that stopped the
  run. The real drone has no raw laser reading, so this matters there.
- **"Land where it is" lands on the boulder that caused the problem** when the drone was stopped
  above it (flights 2 and 4). In the simulator the drone passes through boulders, so this shows
  as a touch; on the real course the drone would come down on top of a 53 cm boulder. See "What
  is left".
- **Raising the gate to 3 is not a sure fix.** It did not help when the drone stopped above a
  boulder (flight 5) or crossed one at speed (flight 7). At the window the same flight went
  well once (flight 8) and badly once (flight 6), where it let PX4 take the window's wall for the
  floor.

## The laser check setting, in plain words

PX4 compares every new laser reading with what it expected from the drone's motion. If the
difference is too big, it treats the reading as suspect, and if that happens while the drone is
moving slowly, it stops using the laser for height until the drone climbs or descends quickly,
which our flights never do. `EKF2_RNG_K_GATE` sets how big a difference PX4 tolerates. Drone 16
uses 1, PX4's default; PX4 allows 1 to 5. The difference is compared as a square, so 3 makes a
sudden jump count 9 times less.

- **What it does:** with 3, the jump as the laser passes over the window's 81 cm lower wall no
  longer makes PX4 drop the laser at the next slow moment. In three window flights (two earlier
  in the day, and flight 8) it kept the height error near 10 cm instead of 17 to 53 cm.
- **What it costs:** PX4 also accepts bigger genuine jumps, so when the laser sees the top of a
  wall or a boulder, PX4 is more likely to take it as a change in the floor. In one earlier
  window flight this left a lasting 10 to 15 cm offset, and in flight 6 PX4 kept the wall's top
  as its floor after the wall, so its height went 78 cm wrong and the run stopped. It does not
  help when the drone stops above a boulder (flight 5) or crosses one at speed (flight 7).
- **When it is needed:** only for the window. The side-gap route, which the first run uses,
  never crosses the window's wall, and with drone 16's settings it finished cleanly with the
  boulders where our maps say (flight 3, and 107 s, 7 cm earlier). A surprise boulder on the
  side-gap route upsets PX4 with or without the gate (flights 1 and 7); the checks handle that.
- **How to change it, and back:** in QGroundControl, connected to drone 16, open Vehicle Setup
  (the gear icon), then Parameters, search for `EKF2_RNG_K_GATE`, type the new value, and save.
  Write down the old value (1) first. Restart the flight controller afterwards and check the
  value again, so you know it is the one in force. To change it back, do the same with 1.
  Section 12 of `ARCHITECTURE.md` suggests asking the organizers first; rule 8.5 doesn't list it
  as a protected safety setting.

We do not recommend changing it on the real drone today. The first run uses the side gap, which
doesn't need it.

## What is left

1. **Don't land on the boulder that caused the problem.** When the run stops for a height
   problem while the drone has been stopped, it should first move off that spot, for example
   back to the previous corner of the route, where it already flew without trouble, and land
   there. When the problem came after crossing a boulder at speed (flight 1), moving back would
   cross it again, so the rule has to tell the two cases apart. Not written; it changes the
   competition program's behaviour, so the team should decide.
2. **Camera tracking's sideways drift can bring a propeller onto a wall** (flight 5, the C1
   chicane wall after the tunnel, 12 cm of drift against 35 cm of planned room). This leg is on
   the side-gap route too. Correcting drift against the map (section 5) is not written yet.
3. **Climbs trail by up to 12 cm.** PX4's height lags the true height while climbing, and the
   drone overshoots by about 7 to 10 cm. At the window, the route follower moves on once camera
   tracking says the drone is within 10 cm of the window's height, even while it is still
   rising. In flight 6 the drone crossed 8 cm high, 6 cm from the frame; in flight 8 it crossed
   at the middle. Waiting at the top of the climb until the drone has stopped rising would be
   safer. That wait belongs in `route_follower.py`, which this work could not edit (below).
4. **On the real drone, before any flight:** confirm that `/fmu/out/estimator_status_flags`
   arrives (the flag check needs it) and that camera tracking's height stays within a few
   centimetres of a tape measure over a few metres (the new camera check needs it). Drone 16
   does not publish `/fmu/out/distance_sensor`, so the raw-laser check will not run there.
5. **The decision on `EKF2_RNG_K_GATE`** for later runs through the window (above).

### Changes needed in files this work could not edit

- `drone_code/team_nav/README.md`, in the paragraph on stops and boulders: replace "The window's
  line-up spot can't be moved: the safety net is right behind it, and the window has to be
  crossed straight through its middle. So the window route still stops 0.13 m from the boulder
  in front of the window." with "The drone climbs to the window's height on the spot, 0.56 m in
  front of the wall, so both of its stops at the window keep at least 0.40 m from every
  boulder (opening_crossings.py)." In the table above it, the three "window first" rows become
  (length, least room, closest moving, closest stop):
  drawing 17.2 m, 0.38 m next to the first maze wall, 0.17 m just past the window, 0.41 m;
  phone scan 18.3 m, 0.35 m next to the third maze wall, 0.09 m just past the window, 0.38 m;
  scan trusting every wall 18.4 m, the same as the scan otherwise.
- `drone_code/team_nav/route_follower.py`, at the window: after the straight-up climb, the
  target moves on as soon as the drone is within 10 cm of the window's height, even while it is
  still rising, so it can cross up to about 8 cm high (flight 6). Suggested change: at a corner
  reached by a straight up-or-down leg, also wait until the drone's upward speed is below about
  0.05 m/s. Not tested.
- `simulator/test_flights/README.md`, "What makes a flight fail", last bullet: "PX4's height
  estimate strayed more than 10 cm from the true height in flight, or more than 20 cm during the
  takeoff climb. The time on the floor after touchdown is not counted."
- `simulator/test_flights/README.md`, "What makes a flight fail", last bullet: "PX4's height
  estimate strayed more than 10 cm from the true height in flight, or more than 20 cm during the
  takeoff climb. The time on the floor after touchdown is not counted."
- `docs/ARCHITECTURE.md` section 11, the bullet "PX4's height going wrong stops the run": add
  that the check also compares PX4's height with camera tracking's (30 cm for 1 s) and runs
  until touchdown, and that without the raw reading it watches PX4's own laser flag.

## Where everything is

| What | Where |
|---|---|
| Planner and checks | `drone_code/team_nav/team_nav/` (its README lists the files in reading order) |
| The safety rules | `run_safety.py`, `height_check.py`, `touch_detector.py` |
| The landing and its checks | `flight_steps.py` (`land`, `DescentHeight`, `SafetyWatch.check_during_landing`) |
| The window crossing | `opening_crossings.py` |
| Tests | `drone_code/team_nav/test/` |
| Running the real program in the simulator | `simulator/test_flights/real_program_test.py` |
| The referee's height and boulder watch | `simulator/ros_packages/course_simulator/course_simulator/boulder_watch.py` |
| Surprise worlds | `course_map_tools/make_boulder_surprise_worlds.py` (regenerate with the command in its header) |
| Each flight's records | `simulator/generated_files/team_runs/` (the program) and `flight_records/` (the referee) |
| Flight results and reasoning | `docs/ARCHITECTURE.md` section 7 |
