"""Native attitude provenance used by the opt-in launch transition guard."""
import math


def accept_attitude(vehicle, message, clock, now_ns):
    """Store only source-timed unit attitude; invalid messages remove admission."""
    try:
        q = tuple(float(value) for value in message.q)
        reset = message.quat_reset_counter
        capture = clock.capture(message.timestamp_sample, now_ns)
        if (len(q) != 4 or not all(math.isfinite(value) for value in q)
                or abs(sum(value * value for value in q) - 1.) > 1e-3
                or type(reset) is not int or not 0 <= reset <= 255):
            raise ValueError('invalid_attitude')
        previous = vehicle.attitude_capture_ns
        if previous is not None and (capture < previous or
                (capture == previous and (vehicle.q != q or vehicle.attitude_reset_counter != reset))):
            raise ValueError('attitude_sample_rewound_or_conflicting')
    except (ValueError, TypeError, AttributeError, OverflowError):
        vehicle.q = None
        vehicle.attitude_capture_ns = vehicle.attitude_received_ns = vehicle.attitude_reset_counter = None
        return 'launch_attitude_invalid'
    vehicle.q, vehicle.attitude_reset_counter = q, reset
    vehicle.attitude_capture_ns, vehicle.attitude_received_ns = capture, now_ns
    return None
