"""Readings the tunnel_first mission takes from PX4's reports and camera tracking, each one a small
function of the link (flight_steps.DroneLink), returning None when the report is missing.

REAL DRONE CODE, also used in the simulators. It only reads.

Words: "heading" is the way the nose points, seen from above. PX4 writes it from north, clockwise
("NED"); camera tracking and flight_interface write it from the x axis toward y ("ENU");
px4_heading_degrees converts PX4's to camera tracking's way (heading_turn.py explains).
"""
import math

from team_nav.heading_turn import odom_heading_from_px4_heading, wrapped_degrees


def px4_heading_degrees(link):
    position = link.reports.latest.get("px4_position")
    heading = None if position is None else getattr(position, "heading", None)
    if heading is None or not math.isfinite(float(heading)):
        return None
    return math.degrees(odom_heading_from_px4_heading(float(heading)))


def camera_heading_degrees(link):
    heading = link.tracked_heading_radians()
    return None if heading is None else math.degrees(heading)


def heading_split_degrees(link):
    """PX4's heading minus camera tracking's, or None if either is missing."""
    px4, camera = px4_heading_degrees(link), camera_heading_degrees(link)
    return None if px4 is None or camera is None else wrapped_degrees(px4 - camera)


def px4_speed_metres_per_second(link):
    position = link.reports.latest.get("px4_position")
    if position is None:
        return math.inf
    return float(math.sqrt(position.vx ** 2 + position.vy ** 2 + position.vz ** 2))


def yaw_rate_degrees_per_second(link):
    """PX4's own turn rate about the vertical (vehicle_odometry), else the total rotation rate."""
    odometry = link.reports.latest.get("px4_odometry")
    if odometry is not None:
        return math.degrees(float(odometry.angular_velocity[2]))
    return link.rotation_degrees_per_second() or 0.0


def px4_reset_counters(link):
    """(heading resets, sideways position resets, attitude resets), each None if not reported."""
    position = link.reports.latest.get("px4_position")
    attitude = link.reports.latest.get("px4_attitude")
    return (None if position is None else getattr(position, "heading_reset_counter", None),
            None if position is None else getattr(position, "xy_reset_counter", None),
            None if attitude is None else getattr(attitude, "quat_reset_counter", None))


def px4_position_field(link, name):
    position = link.reports.latest.get("px4_position")
    return None if position is None else getattr(position, name, None)


def px4_estimator_flag(link, name):
    """One of PX4's estimator_status_flags (cs_rng_hgt, cs_rng_kin_consistent, ...), as a bool, or None."""
    flags = link.reports.latest.get("estimator_flags")
    value = None if flags is None else getattr(flags, name, None)
    return None if value is None else bool(value)


def px4_uses_camera_heading(link):
    return px4_estimator_flag(link, "cs_ev_yaw")


def px4_reports_ground_contact(link):
    landed = link.reports.latest.get("px4_land_detected")
    if landed is None:
        return True
    return bool(landed.landed or getattr(landed, "ground_contact", False) or getattr(landed, "maybe_landed", False))


def camera_height_metres(link):
    """Camera tracking's height up from the launch mark, or None."""
    from_launch = link.position_from_launch()
    return None if from_launch is None else float(from_launch[2])


def px4_height_metres(link):
    """PX4's height up from the launch mark, or None (flight_steps.DroneLink measures it)."""
    return link.px4_height_risen_metres()
