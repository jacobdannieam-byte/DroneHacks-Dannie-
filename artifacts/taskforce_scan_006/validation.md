# Optional waypoint scan validation

Base revision: `4b1009e`, delivery branch `code-integration-hybrid`.

- Full repository: `python -m pytest tests/ -q -p no:cacheprovider` — **763 passed in 37.89 s**. TEMP/TMP were set to the workspace sibling `taskforce-scan-006/test-temp` to avoid the previously observed Windows sandbox TEMP rename restriction.
- Worker focused scan/runtime/reobservation/Gazebo/gateway checks: **82 passed in 7.85 s**.
- Independent reviewer: scan and runtime tests **32 passed in 5.63 s**, no remaining actionable finding for shadow-only publication.
- Existing baseline before integration: strategy portfolio 8 passed; runtime/planner 30 passed.
- `git diff --check` passed.

The positive scan trace injects synthetic capture-aligned depth/state and synthetic free volume at the H1-to-runtime seam. It exercises real course scheduling, 24 measured yaw sectors, swept checks and checkpoint transitions. It is not native sensor evidence, an autonomous yaw flight, proof of complete sensor coverage or a measured stopping envelope. Negative cases cover freshness, drift, rate, reference reset, deadlines, clearance and native-owner rejection.

Scans are opt-in and remain shadow-only. Initial scan requires measured stability; selected-waypoint scans do not advance a checkpoint until the prescribed observation sequence completes. Completion does not refresh old map evidence or assert whole-body clearance. Default mission behavior and protected Classic are unchanged. No simulator, command owner, arming or flight was started for this change.
