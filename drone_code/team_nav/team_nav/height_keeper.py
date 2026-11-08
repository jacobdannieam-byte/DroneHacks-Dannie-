"""Keeps the drone at its true flying height when PX4's own height slowly goes wrong, using
camera tracking's height. REAL DRONE CODE, also used unchanged in the simulator.

Why it is needed
----------------
PX4 works out its height mostly from the bottom laser. When the drone flies over a boulder, the
laser's reading suddenly drops by the boulder's height, PX4 may stop trusting the laser, and its
height then drifts. PX4 holds the height it believes, so the drone really sinks or climbs. In the
Gazebo run of 2026-11-08 the drone sank from 0.82 m to the floor in 4 s this way.

What it does
------------
flight_interface passes our target height straight to PX4. So if PX4 believes the drone is
higher than it really is by some amount, we ask PX4 for that much more height, and the drone
ends up where we want it. That amount is the "height correction". Each cycle, this file compares
camera tracking's height with the wanted height and moves the correction slowly toward closing
the gap (GAIN_PER_SECOND), ignoring gaps below DEADBAND_METRES. The correction never exceeds
LARGEST_CORRECTION_METRES. run_safety.py knows the correction: it still lands the drone if PX4
and camera tracking disagree by more than its limit beyond what we are correcting.

What it does not do: send targets itself, or work during the climb and the landing (only live
navigation uses it).
"""

# Our own choice. Gaps smaller than this are noise between the two height readings.
DEADBAND_METRES = 0.03

# Our own choice. A 10 cm gap changes the correction by 10 cm a second. Fast enough for the
# slow drift after a boulder, slow enough that a jumpy camera height doesn't shake the drone.
GAIN_PER_SECOND = 1.0

# Our own choice. Beyond this, something bigger is wrong; run_safety.py then lands the drone.
LARGEST_CORRECTION_METRES = 0.3


class HeightKeeper:

    def __init__(self):
        self.correction_metres = 0.0

    def update(self, wanted_height_metres, camera_height_metres, seconds_since_last_update):
        """Returns the new correction: how much higher than wanted to ask PX4 for."""
        gap_metres = wanted_height_metres - camera_height_metres
        if abs(gap_metres) <= DEADBAND_METRES:
            return self.correction_metres
        self.correction_metres += GAIN_PER_SECOND * seconds_since_last_update * gap_metres
        self.correction_metres = max(-LARGEST_CORRECTION_METRES, min(LARGEST_CORRECTION_METRES, self.correction_metres))
        return self.correction_metres
