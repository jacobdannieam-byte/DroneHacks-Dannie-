# How we debug drone 16

This is the team's method. Use it for every flight problem. Keep each step short.

## The order

1. **Get real data first.** Fly a short smoke test, or use the records of a flight that already
   happened (`flight_records_from_drone/`, copied with `drone_code/collect_flight_records.sh`).
   Real data is the truth. The simulator is only a tool.
2. **Compare the data with what you saw.** Write down what the team saw by eye (for example,
   "it slid back while it turned left"). Then find that moment in the data. Do not explain a
   problem until the data shows the same thing as the eyes.
3. **Break the problem down, one link at a time.** Follow the chain and find the first link
   that is wrong:
   - what our program sent (`fmu__in__trajectory_setpoint`, `skybeta__setpoint__pose`);
   - what PX4 believed (`fmu__out__vehicle_local_position`, `fmu__out__estimator_status_flags`);
   - what camera tracking gave PX4 (`fmu__in__vehicle_visual_odometry`, its delay);
   - what really happened (camera tracking, video, the team's eyes).
4. **Check by hand.** A teammate can move and turn the real drone with the motors disarmed.
   This tests a hypothesis in one minute with no risk (for example, turn the drone through
   a full circle and see if PX4's heading follows the camera's).
5. **Then change the code.** Check the change on the recorded flights first, then with the
   unit tests. Use the Gazebo simulator only to confirm, and only for what it models well.
   The simulator must match the real drone. A problem that only the simulator has is noted,
   not chased.

## Fast feedback loops

Use the shortest test that answers the question. A long test needs a stated reason.

| Question | Fast test | Time |
|---|---|---|
| Would a new safety rule have caught a real problem, and left good flights alone? | Replay it on recorded flights (the replay script runs the real `run_safety.py` code on the recorder's data) | seconds |
| Is the logic of a rule right? | Unit tests: `test/test_run_safety.py`, `test/test_height_check.py` (plain Python, no ROS) | under 1 s |
| Does PX4 follow the camera when the drone turns? | Hand test, disarmed: turn the drone a full circle | 1 min |
| How late are camera tracking's reports right now? | Measure on the drone for 60 s | 1 min |
| Does the whole program still fly the course? | Gazebo full course | 6 min: only before a real flight |

Rules of thumb:
- Before any test longer than about 1 minute, say which question it answers and why no
  shorter test can.
- Problems that come in bursts (such as the camera delay) need several short samples, not one
  long one.
- Stop a test as soon as it has answered the question.

## Example: the drift of 2026-11-08

- Seen: the drone slid while it turned, and sometimes moved the wrong way.
- Data: PX4 often stopped using camera tracking (`cs_ev_pos`, `cs_ev_yaw` off). It then used the
  compass, which pointed 135 to 180 degrees away from the camera's heading.
- Chain: camera reports came 200 to 400 ms late instead of 60 ms, PX4 dropped them, and its
  heading swung to the compass. The swings came right after arming and in the first 0.3 m of
  the climb.
- Check: replaying the new safety rule (run_safety.py rule 5) on four recorded flights showed it
  left the good 05:04 flight alone and stopped the three bad ones before they slid.
- Root cause of the late reports, found the same day with 60 s tests on the floor: our two
  programs that read the 2 MB camera pictures (depth_camera, flight_recorder) took them through
  shared memory, which is too small for one picture. The camera program waited for our programs
  and camera tracking got its pictures late (25% of reports over 180 ms). With
  `team_nav/transport/pictures_over_network.xml` they read over 127.0.0.1 instead: 0.1% and
  0.0% in two tests, with both programs running. The language (Python) was not the cause.
