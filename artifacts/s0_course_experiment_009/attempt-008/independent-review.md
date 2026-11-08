# Independent attempt 008 review

Attempt 008 actually lifted off, then the live occupied-voxel check refused publication during climb. The owner withdrew. The aircraft subsequently returned to ground and disarmed through native behavior; **no LAND command was recorded**. No route checkpoint was completed.

Independent read-only decoding found these native source events:

| Event | Source time |
| --- | --- |
| Offboard and ARM commands | 57.740 s |
| Armed, Offboard/nav 14 | 57.752 s |
| Landed flag becomes false | 59.612 s |
| Armed, nav state 2 | 62.192 s |
| Landed flag becomes true | 66.108 s |
| Disarmed, nav state 14 | 68.120 s |

The only vehicle commands were 176 (Offboard request) and 400 (ARM). Final landed/disarmed state must not be described as a commanded LAND recovery or assumed Position-mode recovery. The retained native fallback code is 2.

The external referee's 4,502 samples show peak launch-relative height **0.2262 m at 62.336 s**, after the approximately 61.2 s publication refusal. Course X ranged from −2.5146 to −2.5125 m and Y from 0.9144 to 1.0865 m, a northward displacement of approximately 0.1721 m. The referee reports zero checkpoint crossings and zero counted obstacle touches, with no course exit or above-wall event. This does not establish absence of every physical contact, especially intended ground contact.

For the 57–80 s native position window, 2,300 samples show NED north [−0.003694, 0.184159] m, east [−0.001051, 0.006829] m, down [−0.198869, 0.033506] m. Peak native estimated 3D speed was **0.855546 m/s at 62.748 s**, after withdrawal. The earlier C1 calibration speed limit was not an acceptance limit for this separately authorized unqualified experiment; the measured speed is nevertheless retained rather than omitted.

The finalized raw recording covers 106.572 s and passed its integrity check. `independent-review.json` includes its independently computed SHA256, exact transitions, command rows and referee result. `ordered-native-audit.json` and `ordered_native_audit.py` preserve the decoded ordering evidence. Decoding used the pinned image, network disabled, a read-only evidence mount and no ROS initialization. This reviewer launched no simulator or command publisher.

The veto demonstrates that a live occupied cell blocked this startup command. These records alone do not establish whether that cell represented an obstacle, floor return, accumulated mapping error or another cause; geometric diagnosis must inspect the actual cell and source-frame evidence. The result is a retained short-flight failure, not full-course or moving S0-policy success.
