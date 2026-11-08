# Independent attempt 003 review

This attempt remained disarmed and physically stationary. Independent finalized-bag decoding found 284 native status reports, all arming state 1, zero vehicle commands, and 20 trajectory-setpoint/20 Offboard-mode messages used for interrupted priming. No ARM command was issued. External referee CSV positions stayed exactly at course X −2.5146 m, Y 0.9144 m, height 0 m.

Raw bag SHA256: `aa46d38e5b1078880bbb9b7f0591ab2dd21085c7e17ec86512bfedb5ab510fee`. SQLite integrity and identical pre/post-decode hashes passed. The decoder ran without ROS initialization, with network disabled and evidence mounted read-only. Details are in `independent-review.json` and `review_raw.py`.

Owner records show 13 wait-to-arm priming transitions between ROS 109.800 and 115.600 s, followed by returns to wait after approximately 52–100 ms. The last emitted target was at 115.652 s. These intervals never completed continuous priming. The final retained state was preflight, not a successful flight or course result.

Raw local-position source timestamps were ahead of bag receipt timestamps for 200 samples, by up to 32 ms; 12 status reports were ahead by 4 ms. Other local-position receipts lagged source by up to 20 ms. These are recorder timing observations, not the owner's exact callback schedule. They demonstrate the bounded positive-skew condition exists, but cannot uniquely establish which callback caused each readiness drop.

Static inspection found a second concrete readiness path: H1 `prepare()` unconditionally returned `awaiting_capture_pose` whenever a newer depth frame was pending, even if the previous synchronized snapshot remained valid under its original age and tracking checks. The proposed experimental opt-in retains that previous snapshot only while all existing temporal, tracking, health, reset and geometry checks still pass; timestamps must not be refreshed. Direct-owner bounded-future handling similarly must preserve only a still-fresh previously admitted report, never admit the future sample or hide a contradictory validity/authority/reset report.

The attempt did not record per-tick component readiness reasons, so exact attribution between these paths remains unresolved from this evidence. New readiness diagnostics are needed to distinguish them on subsequent authorized experiments. This review authorizes no relaunch and claims no moving-policy performance.
