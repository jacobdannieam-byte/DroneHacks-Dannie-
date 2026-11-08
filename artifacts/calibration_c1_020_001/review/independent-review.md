# Independent C1 result review

**C1 failed its navigation-height requirement before the braking trigger. Native LAND recovery succeeded within the approved bounds.** No braking measurement, retry, R1 execution or reusable stopping qualification follows from this attempt.

The reviewed source was `2ecbee3281727cff8c3791068f3503e66edf2bcf`, archive `87e8781e0e26492b0f77e733a0d54cdd6bfc17c7e3016947c9538f5cff350e88`, approved C1 profile `cb0f11b36c8591eacfbdc832b8d080d70c4e6858931d2a7637764be6255ea76c`, domain 57. Approval and runner were independently checked before launch. This reviewer started no simulator or control publisher.

The finalized raw bag was independently deserialized in the pinned image with network disabled and a read-only evidence mount. No ROS node was initialized. SQLite integrity and unchanged raw SHA were checked before/after decoding. Raw bag SHA256: `b00189fc42df0cfd67148a0ef769b1cf1607baadc2e0352b3c53771b9e530d9c`. Detailed results and decoder are adjacent in `c1_raw_review.json` and `c1_raw_review.py`.

There were 3,213 full-rate position samples from ARM through native disarm, with 8–16 ms source gaps, no invalid state or reset-counter change. Native status gaps were 12–504 ms with no conflicting duplicates. All 4,072 strict recorder position/velocity rows matched raw source samples. Its missing brake event and `recording_complete=false` correctly prevent stopping-measurement acceptance.

| Event | Native source time |
| --- | --- |
| ARM command | 22.440 s |
| Zero vertical target after climb | 37.984 s |
| North velocity 0.20 m/s starts | 39.024 s |
| First full-rate estimated height below 0.80 m | 44.960 s |
| LAND command and recorder event | 44.996 s |
| Accepted LAND acknowledgment | 45.004 s |
| Armed AUTO_LAND | 45.016 s |
| Native landed report | 54.560 s |
| Native disarm | 54.568 s |

The owner's later LAND acknowledgment time, 45.028 s, is its reception time, not the raw acknowledgment source timestamp. LAND-to-disarm was 9.572 s, within the 30 s bound. Commands were only Offboard mode, ARM and LAND. The decoded setpoint sequence was zero velocity, vertical climb −0.20 m/s NED, zero velocity, then north +0.20 m/s with zero vertical velocity. No target followed LAND. The north brake trigger of 1.50 m was never reached; north was approximately 1.11 m when LAND was requested.

Estimated height during north transit fell from a maximum 0.866180 m to 0.799848 m. The first estimated breach preceded LAND by 36 ms. External observer samples independently show height about 0.87214 m near transit start and 0.792942 m immediately before LAND. The observer is sampled external validation, not an input to the flight algorithm or a contact-force measurement.

All-phase maximum native estimated speed was 0.329345 m/s; sampled observer maximum was 0.332576 m/s, both below 0.50 m/s. Native NED bounds were north [−0.001443, 1.211966] m, east [−0.003330, 0.005814] m, down [−0.866394, −0.004539] m, within the global approved lane box. The observer reported no side/roof envelope intrusion or center below modeled floor; minimum sampled side/roof sphere margin was 1.65 m. Floor-envelope overlap at intended takeoff/landing contact is not treated as collision-free evidence.

The source explains the limited height reserve without requiring a speculative PX4 defect: `refnav/calibration_trial.py` enters its zero-vertical-velocity settling state as soon as height enters the broad 1.0 ± 0.2 m band. It does not require reaching the 1.0 m center. Transit then commands `(0.20, 0, 0)` velocity without height-error feedback. This attempt therefore began translation with only about 6.6 cm of estimated lower-band reserve, which the recorded drift consumed. The height gate correctly stopped the experiment rather than widening the acceptance band. A future design may separately tighten center-height acquisition and evaluate bounded altitude regulation; either would change the experiment and require disarmed checks and new approval. This review does not authorize those changes or another attempt.
