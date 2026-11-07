# Depth / VIO handoff (branch `soham-depth`)

Findings from reading `onboard/slam`, `onboard/vio`, `refnav/depth.py` and the parts of
`refnav/core.py` that consume them. Behaviour was checked with synthetic tests; nothing was flown.

## What changed on this branch

- `refnav/depth.py`
  - New `four_clearances(frame, floor_down_m, ...)` returns `ahead / left / right / low` from one depth frame.
    Each value is `None` (unknown: never move on it), `inf` (observed clear at the judged cross-section) or a
    distance in metres. Each `Probe` also gives `obstacle_m` (evidence you may stop on), `seen_from_m` (blind
    zone) and `why`.
  - Sides default to ±20° with a 0.4 m half width. ±60° is an option, but a forward camera with about ±35°
    usable field of view never sees that corridor, so it always returns `None`.
  - `low` covers floor + 0.15 m up to the bottom of the flight band, which holds the 18 in boulders. It needs
    `floor_down_m` (body height above the floor); without it, `low` is `None`.
  - `DepthFrame` adds `window_in_view()` and `seen_from()`. `corridor_coverage()` gains `center_down_m` and
    `reach`; with their defaults it returns exactly what it did before.
  - `clearance()` is unchanged.
  - Bug fix: 16UC1 `65535` now decodes as no return (it used to be a valid 65.5 m return).
- `tests/test_depth_clearances.py`: 27 synthetic-frame tests on the 160×100 test camera and the 640×400 sim
  camera. Removing any one gate makes at least one of them fail.
- `refnav/core.py` and `refnav/node.py` are **not** changed. Nothing calls `four_clearances` yet.

## Gaps for the owner to act on

1. **The feature count is never checked by `ev_bridge`.** It subscribes only to
   `/visual_slam/tracking/odometry` and never reads `/visual_slam/status`, `/visual_slam/vis/observations_count`
   or `/visual_slam/processing_status`. Normal tracking sees about 184–270 features. The node's own threshold,
   `min_visual_obs: 10`, is very low.
2. **IMU-only poses are reported healthy.** With `imu_only_policy: publish` (`onboard/config/vio.yaml`),
   cuVSLAM publishes IMU-propagated poses as normal odometry when vision fails. The bridge forwards them to PX4
   with `/estimator/healthy = true`, because slow drift (under 1 m per step and under 5 m/s) passes
   `VioHealth`. This will happen in the dark tunnel.
   - Fix options: set `imu_only_policy: withhold`, or gate the bridge's health on the observation count.
3. **A single 1 m jump causes a landing.** A step over 1.0 m or over 5 m/s marks the sample insane. The bridge
   then withholds odometry and publishes healthy = false for 5 s (`health_quiet_s`), and refnav lands after
   1.0 s unhealthy (`health_bad_land_s`). This includes a legitimate relocalization snap, and VIO coming back
   after a dropout more than 1 m off. The jump limit does not scale with the time gap.
4. **The drone can hover with no timeout** (`core.py`). A frame that is usable (at least 30% valid pixels)
   but leaves the corridor unobserved (coverage below 0.6) still refreshes `depth_t`. The 3 s depth-lost
   landing therefore never fires, `_clearance` holds, and the drone hovers in `go`/`check` indefinitely. A pure
   test reproduced it: still airborne after 200 s. It needs a timeout that falls back to land.

## Smaller items

- When tracking is lost (no pose), health goes false 0.5–0.7 s later and the drone lands about 1.5–1.75 s
  after the last sample.
- If end-to-end VIO latency exceeds 0.3 s (`max_sample_age_s`), every sample is stale and health never turns
  true.
- At startup, the bridge's 25 s sole-publisher check keeps health false. If it sees a second publisher, it
  raises and the bridge process exits.
- In the `vio` services profile, `/estimator/healthy` comes only from the bridge. It means "VIO fresh and
  sane", not "PX4 fused it".
- Existing `depth.py` traps that `four_clearances` closes, but `core.py`'s current check still has:
  - Coverage counts the crossing rays, not how much of the window is in view.
  - A surface nearer than 0.3 m to the camera reads as clear.
- `min_obstacle_px = 1200` gives `min_hits` 19, which was tuned for 1280×800. On the 640×400 sim, a 5 cm pole
  is missed beyond about 2.2 m. About 5 hits would restore the intended range.
- `node.py` levels each frame with the latest attitude, not the attitude at capture time.
- The low band tolerates about ±2° of attitude error. At 3° the floor appears as a fake obstacle at about
  3.7 m.

## Limits of one forward frame

- At `HEIGHT_M` 1.2 (about 1.38 m above the floor), a boulder leaves the image within about 2.5 m, and `low`
  is judged at about 3.1–3.25 m. The planner must remember obstacles seen earlier.
- The ahead corridor is fully in view only from about 1.1 m (`stop_m` is 0.9).
- In the dark tunnel all four values are `None` (passive stereo, no projector). That stage needs a non-depth
  strategy.
- A textureless object covering less than 40% of the window can still read as clear, same as today.
- All thresholds come from ideal synthetic frames. Check them on recorded frames before trusting them.

## Wiring sketch (for whoever owns `core.py`)

```python
hits = max(3, round(p.min_obstacle_px / p.depth_stride ** 2))
c = D.four_clearances(self.frame, floor_down_m, points=self.depth_pts, bearing_rad=bearing,
                      half_width_m=p.corridor_half_m, band_m=p.band_m, min_hits=hits,
                      min_coverage=p.min_corridor_coverage, min_valid_frac=p.min_valid_frac,
                      at_m=p.stop_m + p.lead_max_m)
```

`floor_down_m` is the VIO height above home plus the landed body-origin height (0.18 m in sim; measure it on
the real airframe), cross-checked against PX4 `dist_bottom` from the VL53L1X. Pass `None` if they disagree by
more than about 0.1 m.
