"""Makes the fallback missions hold camera tracking's height rather than PX4's, within strict limits.

REAL DRONE CODE, also used in the simulator and the tests. fallback_run.py wraps the route follower
in HeightHeldByCameraTracking, and fallback_abort.py uses it for the flight to a landing spot.

Why
---
PX4 (the flight controller's software) flies to its own height estimate. Up high, PX4 stops using
the bottom laser when a wall top passes under it, and then uses its air pressure sensor, which
drifts. In the simulator on 2026-11-08 the drone rose from 2.2 m to 2.7 m in 12 s after the second
maze wall while PX4 still believed 2.3 m: 0.35 m from the net roof. Camera tracking measured the
true height all along.

How
---
PX4 reads each target's height in its own terms. So each target's height is moved by the
correction: an estimate of the gap between PX4's height and camera tracking's (both measured from
the launch mark). The follower itself still works on camera tracking's positions.

The limits (the team's review, 2026-11-08), because camera tracking can be wrong too:
  - The correction is never more than LARGEST_DOWNWARD_CORRECTION_METRES down or
    LARGEST_UPWARD_CORRECTION_METRES up, so a target is never more than 0.15 m above the route's
    height, whatever camera tracking says. (The route's highest point is 2.3 m, so the highest
    target is below run_safety.py's 2.45 m limit and far below the 3.05 m net roof.)
  - The correction changes by at most CORRECTION_RATE_METRES_PER_SECOND, so a sudden error in either
    height can only move the drone slowly.
  - A camera height that differs by more than CAMERA_HEIGHT_STEP_LIMIT_METRES from the last one
    accepted is ignored, and so is every later one until camera tracking comes back to within that
    of the last accepted height. A jump in camera tracking therefore freezes the correction.
  - A stale camera height (no report for 0.5 s, or tracking lost) is ignored too.

What it does not do: decide that a height is wrong and stop the run (height_check.py does that).
"""
from team_nav.run_safety import CAMERA_TRACKING_LONGEST_SILENCE_SECONDS

# The team's limits (review of 2026-11-08): at most 0.3 m. Our own choice within that: 0.20 m down,
# because a camera tracking error that reads too high lowers the drone, and at 2.3 m minus 0.3 m the
# drone's underside (0.10 m below its centre) would pass only 3 cm over the scan's 1.87 m barrier;
# with 0.20 m it keeps 13 cm. And 0.14 m up, so no target is ever more than 0.15 m above the route.
LARGEST_DOWNWARD_CORRECTION_METRES = 0.20
LARGEST_UPWARD_CORRECTION_METRES = 0.14
CORRECTION_RATE_METRES_PER_SECOND = 0.1
CAMERA_HEIGHT_STEP_LIMIT_METRES = 0.15
# Our own choice: corrections this large are written in the records.
ANNOUNCED_CORRECTION_METRES = 0.10


class HeightHeldByCameraTracking:
    """Wraps a route follower; its targets' heights are moved by the limited correction."""

    def __init__(self, follower, link):
        self.follower = follower
        self.link = link
        self.correction_metres = 0.0
        self.last_accepted_camera_height_metres = None
        self.announced_large_correction = False
        self.announced_ignored_step = False

    @property
    def next_waypoint_number(self):
        return self.follower.next_waypoint_number

    def has_finished(self, drone_position_metres):
        return self.follower.has_finished(drone_position_metres)

    def next_target(self, drone_position_metres, seconds_since_last_target):
        target = self.follower.next_target(drone_position_metres, seconds_since_last_target)
        self.update(seconds_since_last_target)
        target[2] += self.correction_metres
        return target

    def update(self, seconds_since_last_target):
        """Moves the correction towards the measured gap, within the limits."""
        px4_height_metres = self.link.px4_height_risen_metres()
        camera_height_metres = self._usable_camera_height()
        if px4_height_metres is None or camera_height_metres is None:
            return   # Keep the last correction until both heights are usable again.
        wanted_metres = px4_height_metres - camera_height_metres
        largest_change_metres = CORRECTION_RATE_METRES_PER_SECOND * seconds_since_last_target
        change_metres = min(max(wanted_metres - self.correction_metres, -largest_change_metres), largest_change_metres)
        corrected = self.correction_metres + change_metres
        self.correction_metres = min(max(corrected, -LARGEST_DOWNWARD_CORRECTION_METRES), LARGEST_UPWARD_CORRECTION_METRES)
        self._announce(wanted_metres)

    def _usable_camera_height(self):
        """Camera tracking's height from the launch mark, or None if it is stale or has jumped."""
        if self.link.camera_tracking_says_lost():
            return None
        if self.link.camera_tracking_age_seconds() > CAMERA_TRACKING_LONGEST_SILENCE_SECONDS:
            return None
        from_launch = self.link.position_from_launch()
        if from_launch is None:
            return None
        height_metres = float(from_launch[2])
        last = self.last_accepted_camera_height_metres
        if last is not None and abs(height_metres - last) > CAMERA_HEIGHT_STEP_LIMIT_METRES:
            self._announce_ignored_step(height_metres - last)
            return None
        self.last_accepted_camera_height_metres = height_metres
        return height_metres

    def _announce(self, wanted_metres):
        large = abs(self.correction_metres) >= ANNOUNCED_CORRECTION_METRES
        if large != self.announced_large_correction:
            self.announced_large_correction = large
            self.link.log.event("height_correction",
                                f"PX4's height and camera tracking's now differ by {wanted_metres:+.2f} m; targets are "
                                f"moved by {self.correction_metres:+.2f} m so the drone holds camera tracking's height.",
                                gap_metres=round(wanted_metres, 3), correction_metres=round(self.correction_metres, 3))

    def _announce_ignored_step(self, step_metres):
        if not self.announced_ignored_step:
            self.announced_ignored_step = True
            self.link.log.event("height_correction",
                                f"Camera tracking's height stepped by {step_metres:+.2f} m; ignoring it, so the height "
                                f"correction stays at {self.correction_metres:+.2f} m.", step_metres=round(step_metres, 3))
