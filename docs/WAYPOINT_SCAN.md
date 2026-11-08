# Optional waypoint scans in Hybrid shadow

Hybrid shadow can schedule a bounded 360° observation at the initial hover and at selected waypoints. Omit both options to keep the ordinary course behavior. A scan-enabled mission requires `strategy: "hybrid_shadow"` and a planning rate of at least 5 Hz. The native `hybrid_sitl` loader rejects scan-enabled missions until an owner can reconstruct and revalidate the yaw request with qualified clearance and stopping evidence.

In the reviewed mission JSON referenced by `mission_profile`, use `startup_scan_position_m` for the initial hover position and `scan_on_arrival` on each chosen `waypoint` stage. Both options are independent. The following fragment shows the fields on a mission whose map reference and route review are already configured. Its coordinates are illustrative and need to match the reviewed course and hover location.

```json
{
  "startup_scan_position_m": [0.0, 0.1, 1.3],
  "stages": [
    {"stage_id": "observe", "kind": "waypoint", "position_m": [0.4, 0.1, 1.3],
     "scan_on_arrival": true, "arrival_m": 0.20, "timeout_s": 50.0},
    {"stage_id": "land", "kind": "landing", "position_m": [0.85, 0.1, 1.3],
     "landing_site_review": "reviewed_site_id"}
  ]
}
```

The initial scan blocks route planning and waits for a measured stable hover within 0.20 m of its map anchor, including position uncertainty. A waypoint scan starts after two distinct fresh arrival observations, within that stage's arrival tolerance. Each scan needs at least 0.5 s of advancing state samples within the position tolerance, at or below 0.05 m/s and 0.10 rad/s. Stabilization times out after 3 s. The full scan must finish within 35 s on both ROS and monotonic clocks. A waypoint's existing stage deadline also remains in force. After a successful startup scan, the first stage starts its own deadline.

The scan reports 24 successive positive-NED-yaw targets, 15° apart, around the measured anchor. Each target is diagnostic and depends on fresh swept-volume clearance with the configured stopping envelope and an extra 0.10 m of clearance. The scheduler tracks measured yaw across wraparound with a 0.75 rad/s limit, maximum 0.10 m displacement from the anchor, maximum 0.05 m vertical displacement and maximum 0.15 m/s translational speed. A sector counts only after a distinct fresh depth capture with aligned optical azimuth follows the measured settle at that target. Reference changes, missing inputs, excessive drift, rejected clearance and deadlines refuse the scan and block course progress. Each configured scan runs at most once in the course instance for that H1 reference epoch.

The `waypoint_scan` section in the shadow log reports the scan scope, state, sector count, measured turn, last counted depth observation and any current diagnostic target. Every scan record carries `command_authority: false`. A successful shadow replay establishes that the scheduler observed the prescribed turn and views. Map evidence still ages normally, so the report does not establish simultaneous fresh clearance around the vehicle or resolve the near-body blind region. Scan decisions issue no actuator or landing commands.

```powershell
python -m pytest tests/test_waypoint_scan.py tests/test_hybrid_runtime.py -q -p no:cacheprovider
```
