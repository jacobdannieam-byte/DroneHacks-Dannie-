# C1 attempt 001: height-band abort, native landing completed

The single approved `calibration-c1-020-001` attempt failed before the planned braking trigger. It took off, settled and translated north, then the existing navigation-height gate requested ordinary LAND. Native acknowledgement and disarm followed. This is a retained failed measurement, not a stopping-envelope result or successful C1 qualification. No retry, R1 or S0 execution followed.

## Exact execution

User approval: “yes, lets go with it”, following the [batch 008 plan](BRAKING_RECOVERY_APPROVAL_008.md), including its bounded recovery procedure. Runtime source `2ecbee3281727cff8c3791068f3503e66edf2bcf`; archive SHA256 `87e8781e0e26492b0f77e733a0d54cdd6bfc17c7e3016947c9538f5cff350e88`; image `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`; isolated localhost DDS domain 57. The exact approval, runner, profiles and evidence hashes are retained in [the evidence directory](../artifacts/calibration_c1_020_001/).

Fresh focused calibration/command-audit/stopping tests passed: **50 in 0.55 s**. Independent prelaunch review verified the archive, approval, profile, recorder and single-attempt runner. Live preflight passed disarmed/landed/healthy input and idle control-graph checks. The only approved parameter override was ephemeral `MPC_LAND_SPEED=0.3`; its readback was retained. Actual OGRE renderer was D3D12/NVIDIA RTX 5070 Laptop GPU.

## Outcome and evidence

- Owner result: `left_navigation_height_band`; no input fault. LAND issue at ROS 44.996 s, owner receipt of native acknowledgement 45.028 s, native disarm 54.568 s. LAND-to-disarm was 9.572 s.
- The owner issued no `brake_command` event. Its reported peak observed speed was 0.329339 m/s, below the approved 0.50 m/s cap.
- Raw recording finalized with 52.148 s coverage and SQLite integrity PASS. The strict recorder retained 4,072 positions, zero faults and zero pending samples, with LAND and native-disarm events. `recording_complete=false` correctly reflects the missing braking experiment.
- Both standard command-correlation and stopping qualification tools reject the incomplete recording. No distance or latency was promoted into a stopping envelope.
- Recovery probe confirmed landed, disarmed and healthy. No operator console LAND, forced disarm, emergency termination or moving retry was needed. All flight containers stopped.

The external observer is diagnostic evidence only, never a policy input. Floor overlap of the conservative sphere during intended takeoff/landing is not itself a measured collision. The observer has no contact-force stream. Full-rate independent review and its exact metrics are retained with the evidence.

Independent decoding found 3,213 flight-window native positions with 8–16 ms source gaps, no invalid positions or reset changes, and exact correspondence for all 4,072 strict-recorder samples. Translation began at 39.024 s near estimated height 0.866 m. The first below-band native sample was at 44.960 s; height reached 0.799848 m before LAND, with north displacement only approximately 1.11 m. Native peak 3D speed was 0.329345 m/s; sampled observer peak was 0.332576 m/s. The global center box remained intact. Native LAND acknowledgement source time was 45.004 s (owner receipt 45.028 s), AUTO_LAND 45.016 s, landed 54.560 s and disarm 54.568 s. The observer recorded no side/roof intrusion or center below the modeled floor. Its minimum side/roof sphere margin was 1.65 m.

## Diagnosis and next change to review

The calibration driver ends its upward velocity command when height first enters the broad 0.8–1.2 m acceptance band, then waits for low speed. It does not require reaching a narrow neighborhood of the requested 1 m target. During translation it commands `(0.20, 0, 0)` m/s; zero vertical velocity is not closed-loop regulation of altitude back to 1 m. The trace settled near 0.87 m, leaving limited reserve above the lower gate, and subsequently lost altitude. This supports a calibration-controller design issue; it does not establish a PX4 defect.

Next proposed preparation is a narrow, bounded stabilization target around 1 m and explicit reviewed altitude retention during translation, with overshoot/saturation and estimator-disagreement tests. Preserve the existing 0.8–1.2 m abort band and all speed, lane, clock, authority and recovery limits. Any change requires a new frozen source, disarmed checks and a new concrete single-attempt approval before flight. This result does not authorize that repeat or R1. The Hybrid liftoff/contact mechanism was not exercised by this deterministic calibration owner.

## Preserved analysis failures

The first offline audit runner stopped on an unset ROS setup variable under shell `nounset`; the next command audit could not import ROS because its runner replaced `PYTHONPATH`. Both external-runner errors were retained and corrected without changing flight source or raw evidence. Final standard analyzers ran in the pinned image and rejected the incomplete C1 as expected. These offline harness retries are not flight retries.
