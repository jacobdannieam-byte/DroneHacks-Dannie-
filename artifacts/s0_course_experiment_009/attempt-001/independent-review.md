# Independent attempt 001 review

The attempt was refused before command publication. At ROS 27.500 s the mission entered its internal arm phase, but the final geometry check returned `participant_enclosure_intersection`. At 27.552 s it yielded and ended. This internal phase transition was not an ARM command or a flight.

Independent read-only decoding of the finalized bag found 99 native status reports, all arming state 1 (disarmed), and zero vehicle-command, trajectory-setpoint or Offboard-mode messages. Final native telemetry confirmed landed/disarmed. The external referee recorded no completed checkpoints. The companion JSON includes external position ranges; no policy-controlled motion occurred.

Raw SQLite SHA256 is `d27e5c9193565470b342849a9495a9be67627448e78383c66a241562d7566e26`. SQLite integrity and unchanged hash before/after decoding passed. The decoder used the pinned image with no network, a read-only evidence mount, and no ROS initialization.

The nearest decoded native position sample to the refusal was at 27.504 s: NED `[0.00299034, -0.00050337, 0.12364085]` m. The configured zero-origin NED-to-odom transform maps its height to −0.12364085 m, or course height −0.08364085 m after the authored +0.04 m spawn translation. That is below the course floor and explains the enclosure rejection.

The nearest-receipt SIM_ODOM row was received at 27.500 s, with source time 27.436 s and odom position `[0.00145140, -0.00291184, -0.00075871]` m. Its course height is +0.03924129 m. The pair differs vertically by +0.12288214 m. These source times differ by 68 ms; this is a near-failure comparison, not an exact simultaneous estimator-alignment measurement.

Over the full retained bag PX4 NED z ranged from −0.404770 to +0.188249 m, while SIM_ODOM z remained between −0.017744 and +0.016504 m. Therefore the evidence does not establish one constant estimator-origin correction. A correction should bind paired fresh frame observations and account for evolving discrepancy/reset identity, rather than merely relabel the frame or subtract one convenient height. This is a frame/state integration finding, not a renewed calibration prerequisite for the authorized unqualified simulator experiment.

All original evidence remains retained. This attempt supplies no moving-policy, obstacle-avoidance or full-course success result.
