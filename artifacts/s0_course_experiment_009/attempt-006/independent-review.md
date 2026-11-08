# Independent attempt 006 review

Attempt 006 armed on the ground and acquired Offboard, but did not lift off or navigate. The owner's same-timestamp conflict check incorrectly rejected an ordered native status transition. Final native state was disarmed, and the retained referee positions were physically stationary.

Read-only decoding of the finalized SQLite bag found Offboard command 176 and ARM command 400, both issued at 59.620 s and recorded at 59.624 s. Two successive native `vehicle_status` rows then shared source and receipt time 59.640 s: row 36545 reported arming state 1/nav state 4; row 36547 reported arming state 2/nav state 14. Its changed fields were armed time, arming state, latest arming reason, navigation-state timestamp, user navigation intention, navigation state and preflight-check status. This is the native acquisition event that triggered `source_rewound_or_conflicting`; it is not a backward source-clock transition.

| Native transition | Source time |
| --- | --- |
| Armed, Offboard (nav 14) | 59.640 s |
| Armed, nav state 2 | 60.732 s |
| Disarmed, nav state 14 | 70.656 s |

The only recorded vehicle commands were Offboard and ARM. No LAND command occurred, so the final ground disarm must not be reported as a commanded landing recovery. The external referee CSV remained exactly at X −2.5146 m, Y 0.9144 m and height 0 m for every sample: there was no observed lift or translation.

The decoder examined 10,393 local-position, 10,394 attitude, 124 land-detected, 210 status and 103 estimator-flag messages. No nonadvancing source capture appeared in local-position, attitude, land or estimator flags. The status acquisition pair above was the only nonadvancing status pair. Both commands sharing one timestamp are also expected separate command types, not sensor-state conflict.

The appropriate correction is scoped to ordered discrete native status events within one simulation tick. It must preserve immediate invalidation of prior authority when status changes, original timestamps, bounded future-message deferral, and rejection of genuinely backward source time. This evidence does not justify relaxing continuous pose/attitude conflict checks or freshness limits.

`ordered-native-audit.json` preserves exact row IDs, source/receipt times and changed field names; `ordered_native_audit.py` is the read-only decoder. It ran in the pinned image with network disabled and evidence mounted read-only, without ROS initialization. This reviewer started no simulator or command publisher.
