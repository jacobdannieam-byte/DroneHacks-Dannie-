"""The drone's size, and the safety margins the route planner keeps around it.

This runs on the drone and in the simulator. route_planner.py and opening_crossings.py use
these numbers to decide where the drone fits.

The planner treats the drone, seen from above, as a circle around its centre that covers every
propeller tip. Seen from the side, it treats the drone as a slab around its centre height.
"""

# Measured on drone 16: the motor farthest from the centre is 0.15 m back and 0.14 m to the side,
# so 0.205 m away (the motor positions are in simulator/px4_changes/drone_16_settings.sh).
# Assumed: 7-inch propellers, which reach 0.089 m beyond each motor. Nobody has confirmed the
# propeller size. Together that is 0.294 m, rounded up.
DRONE_RADIUS_METRES = 0.30

# Estimated, not measured: the body, propellers and camera reach about this far above and
# below the height the drone reports for itself.
DRONE_HALF_HEIGHT_METRES = 0.10

# Our own choice. Extra room kept beyond the drone's radius, for errors in the map and in the
# drone's idea of where it is. The planner also prefers routes with much more room than this
# whenever it can (see route_planner.py), so this is the least it will ever accept.
SIDEWAYS_SAFETY_MARGIN_METRES = 0.05

# Our own choice. Extra room kept above and below the drone, for height errors.
VERTICAL_SAFETY_MARGIN_METRES = 0.10

# The least room the planner accepts between the drone's centre and anything it could hit.
REQUIRED_ROOM_METRES = DRONE_RADIUS_METRES + SIDEWAYS_SAFETY_MARGIN_METRES

# How far above and below its reported height the drone needs the space to be clear.
HALF_HEIGHT_NEEDED_METRES = DRONE_HALF_HEIGHT_METRES + VERTICAL_SAFETY_MARGIN_METRES


def heights_spanned_metres(flying_height_metres):
    """The lowest and highest heights the drone needs clear when it flies at this height."""
    return flying_height_metres - HALF_HEIGHT_NEEDED_METRES, flying_height_metres + HALF_HEIGHT_NEEDED_METRES
