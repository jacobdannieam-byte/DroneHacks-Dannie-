"""Translate launch-relative targets for the unanchored flight interface.

Used by flight_steps before TargetSender publishes. This pure calculation does
not publish or authorize motion. The interface must have anchoring disabled:
horizontal coordinates use camera tracking, while up uses PX4's height origin.
"""
import numpy as np


def position_target_metres(camera_launch_metres, px4_launch_down_metres,
                           displacement_metres):
    """Return a new target without changing either input array."""
    camera = np.asarray(camera_launch_metres, dtype=float)
    displacement = np.asarray(displacement_metres, dtype=float)
    if (camera.shape != (3,) or displacement.shape != (3,)
            or not np.isfinite(camera).all() or not np.isfinite(displacement).all()
            or not np.isfinite(px4_launch_down_metres)):
        raise ValueError('Launch readings and displacement must be finite three-dimensional positions.')
    target = camera + displacement
    # PX4's vertical coordinate points down; the outgoing target points up.
    target[2] = -float(px4_launch_down_metres) + float(displacement_metres[2])
    return target
