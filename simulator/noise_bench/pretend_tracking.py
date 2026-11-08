"""PRETEND / SIMULATOR ONLY. Never runs on the drone.

Pretend camera tracking (cuVSLAM): when each position report reaches our program and PX4, how
far it has drifted, and whether PX4 is still using it.

How it fits in
--------------
bench_link.py takes a "picture" for camera tracking 30 times a second. For each one it asks
this file when the report arrives (delivery_seconds) and how far tracking has drifted
(drift_metres). Each delivered report is also offered to pretend PX4 (px4_receives_report);
estimate_error_metres() is then how wrong PX4's own position is, which pretend_drone.py flies.

The model, from the 2026-11-08 measurements (understand.json, "latest-real-runs")
------------------------------------------------------------------------------------
  - Pictures are taken on time, 30 a second, and never lost. Only delivery is late.
  - CLEAN state: each report is about 60 ms old when it arrives.
  - BAD state: delivery freezes for 0.20-0.43 s about 67 times a minute; the reports taken
    during a freeze all arrive together when it ends.
  - The two states alternate in blocks of minutes (a "two-state Markov process").
  - PX4 has the compass off (EKF2_MAG_TYPE=5), so there are no heading jumps. PX4 throws away a
    report older than 0.445 s. With no usable report for 0.4 s it stops using the camera
    (cs_ev_pos false) and guesses from its motion sensors, drifting slowly, until a usable
    report comes again; then it snaps back.
  - Tracking itself drifts by about 1 % of the distance flown, in one direction per trial.

What it does not do: lose tracking completely, or jump. It ignores darkness in the tunnel.
"""
import math

import numpy as np

# Measured: CLEAN report age 61 ms median, 76 ms p95.
CLEAN_AGE_SECONDS = 0.060
CLEAN_AGE_SPREAD_SECONDS = 0.008
SMALLEST_AGE_SECONDS = 0.045
# Measured: in BAD episodes, freezes over 200 ms about 67 a minute, up to about 430 ms.
FREEZES_PER_MINUTE = 67.0
SHORTEST_FREEZE_SECONDS = 0.20
LONGEST_FREEZE_SECONDS = 0.43
# Measured: the reports held in a freeze arrive within about 10 ms of each other.
BURST_SPREAD_SECONDS = 0.01
# Measured: BAD blocks lasted 1.3-4.6 min, CLEAN 6-20 min. We use these means.
MEAN_BAD_BLOCK_SECONDS = 180.0
MEAN_CLEAN_BLOCK_SECONDS = 420.0
# Estimated from the measurements: the tracking position wobbles by about 5 mm.
POSITION_JITTER_METRES = 0.005
# Estimated (understand.json): slow drift of about 1 % of the distance flown.
DRIFT_PER_METRE_FLOWN = 0.01

# From PX4 source code and drone 16's settings (understand.json): EKF2_DELAY_MAX 350 ms plus
# 100 ms; older reports are not fused.
PX4_OLDEST_USABLE_REPORT_SECONDS = 0.445
# From PX4 source code (estimated for our version): no usable report for this long and PX4 stops
# using the camera for position.
PX4_DROPS_CAMERA_AFTER_SECONDS = 0.4
# Estimated: while PX4 guesses from its motion sensors alone, its position error grows at a
# small random speed of about this size per axis.
DEAD_RECKONING_DRIFT_METRES_PER_SECOND = 0.05


class PretendTracking:

    def __init__(self, random, starts_bad, start_seconds, longest_seconds):
        self.random = random
        self.freezes = self._make_freezes(starts_bad, start_seconds, start_seconds + longest_seconds)
        self.freeze_index = 0
        self.drift_direction = _random_unit_vector(random)
        self.drift_metres = np.zeros(3)
        self.last_usable_report_seconds = start_seconds
        self.px4_uses_camera = True
        self.dead_reckoning_error = np.zeros(3)
        self.dead_reckoning_speed = np.zeros(3)

    # ---- Freezes -----------------------------------------------------------------------------------

    def _make_freezes(self, starts_bad, start_seconds, end_seconds):
        """A list of (start, end) times when delivery is frozen, for the whole trial."""
        freezes = []
        now = start_seconds
        bad = starts_bad
        while now < end_seconds:
            mean_block = MEAN_BAD_BLOCK_SECONDS if bad else MEAN_CLEAN_BLOCK_SECONDS
            block_end = now + self.random.exponential(mean_block)
            if bad:
                freezes += self._freezes_between(now, min(block_end, end_seconds))
            now = block_end
            bad = not bad
        return freezes

    def _freezes_between(self, start_seconds, end_seconds):
        freezes = []
        mean_gap_seconds = 60.0 / FREEZES_PER_MINUTE - (SHORTEST_FREEZE_SECONDS + LONGEST_FREEZE_SECONDS) / 2.0
        now = start_seconds + self.random.exponential(mean_gap_seconds)
        while now < end_seconds:
            length = self.random.uniform(SHORTEST_FREEZE_SECONDS, LONGEST_FREEZE_SECONDS)
            freezes.append((now, now + length))
            now += length + self.random.exponential(mean_gap_seconds)
        return freezes

    def delivery_seconds(self, picture_seconds):
        """When a report of a picture taken at picture_seconds reaches our program and PX4."""
        age = max(self.random.normal(CLEAN_AGE_SECONDS, CLEAN_AGE_SPREAD_SECONDS), SMALLEST_AGE_SECONDS)
        arrival = picture_seconds + age
        while self.freeze_index < len(self.freezes) and self.freezes[self.freeze_index][1] < picture_seconds - 1.0:
            self.freeze_index += 1
        for start, end in self.freezes[self.freeze_index:self.freeze_index + 3]:
            if start <= arrival < end:
                return end + self.random.uniform(0.0, BURST_SPREAD_SECONDS)
        return arrival

    def seconds_frozen(self, start_seconds, end_seconds):
        """How much of this time delivery was frozen (for the trial's summary)."""
        total = 0.0
        for start, end in self.freezes:
            total += max(0.0, min(end, end_seconds) - max(start, start_seconds))
        return total

    # ---- Drift -------------------------------------------------------------------------------------

    def add_flown_distance(self, flown_metres):
        self.drift_metres += self.drift_direction * DRIFT_PER_METRE_FLOWN * flown_metres

    def reported_position(self, true_position_metres):
        return true_position_metres + self.drift_metres + self.random.normal(0.0, POSITION_JITTER_METRES, size=3)

    # ---- PX4's use of the reports ------------------------------------------------------------------

    def px4_receives_report(self, now_seconds, picture_seconds):
        if now_seconds - picture_seconds > PX4_OLDEST_USABLE_REPORT_SECONDS:
            return
        self.last_usable_report_seconds = now_seconds
        if not self.px4_uses_camera:
            # PX4 takes the camera's position again at once: its own guess snaps back.
            self.px4_uses_camera = True
            self.dead_reckoning_error[:] = 0.0

    def update_px4(self, now_seconds, step_seconds):
        if self.px4_uses_camera and now_seconds - self.last_usable_report_seconds > PX4_DROPS_CAMERA_AFTER_SECONDS:
            self.px4_uses_camera = False
            self.dead_reckoning_speed = self.random.normal(0.0, DEAD_RECKONING_DRIFT_METRES_PER_SECOND, size=3)
            self.dead_reckoning_speed[2] = 0.0    # PX4's height comes from the bottom laser.
        if not self.px4_uses_camera:
            self.dead_reckoning_error += self.dead_reckoning_speed * step_seconds

    def estimate_error_metres(self):
        """How far PX4's own position is from the truth: tracking's drift (PX4 follows the camera)
        plus any dead-reckoning error while PX4 has dropped the camera."""
        error = self.drift_metres + self.dead_reckoning_error
        return np.array([error[0], error[1], 0.0])


def _random_unit_vector(random):
    angle = random.uniform(0.0, 2.0 * math.pi)
    return np.array([math.cos(angle), math.sin(angle), 0.0])
