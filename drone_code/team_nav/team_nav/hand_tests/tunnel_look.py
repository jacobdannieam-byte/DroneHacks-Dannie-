"""Hand test: hold the drone in front of the tunnel and read what the LOOK step would see.

REAL DRONE CODE, run with the PROPELLERS OFF. It only listens: it sends nothing.

    ros2 run team_nav hand_test tunnel_look

Twice a second it prints the opening's width in pixels, its distance, and where the tunnel's
centre is, for example "tunnel centre is 0.12 m to your LEFT", from the edge method and, when
depth_camera is running, from the stereo depth method too (tunnel_look.py explains both). Hold
the drone at about 0.9 m, nose towards the tunnel, 0.8 to 1.9 m from its mouth, and move it
sideways: the offset must follow. Stop with Ctrl+C. It always passes; it is for looking.
"""
import time

from team_nav.hand_tests.hand_test_tools import passed
from team_nav.tunnel_look import depth_left_offset_metres, edges_match_depth, find_opening_edges
from team_nav.tunnel_look_camera import TunnelLookCamera

# Our own choice: twice a second, as the team asked.
SECONDS_BETWEEN_PRINTS = 0.5


def run(listener, person, log):
    camera = TunnelLookCamera(listener.node)
    print("Hold the drone in front of the tunnel, nose towards it. Ctrl+C to stop.", flush=True)
    printed = 0
    try:
        while True:
            listener.wait(SECONDS_BETWEEN_PRINTS)
            line = one_reading(camera)
            print(line, flush=True)
            log.event("tunnel_look", line)
            printed += 1
    except KeyboardInterrupt:
        pass
    return passed(f"Printed {printed} readings.")


def one_reading(camera):
    lens = camera.lens()
    picture = camera.newest_picture()
    if lens is None or picture is None:
        return f"{time.strftime('%H:%M:%S')} waiting for the left picture and its camera_info..."
    fx, cx = lens
    _, gray, _ = picture
    look = find_opening_edges(gray, fx, cx)
    if not look.found_both():
        side = "left" if look.clipped_left else "right" if look.clipped_right else None
        return (f"{time.strftime('%H:%M:%S')} both edges not found"
                + (f"; the {side} edge seems cut off by the picture's border" if side else ""))
    matches, why = edges_match_depth(look, camera.latest_points_from_camera, fx, cx)
    if not matches:
        return f"{time.strftime('%H:%M:%S')} edges at {look.left_column} and {look.right_column} px rejected: {why}"
    text = (f"{time.strftime('%H:%M:%S')} width {look.width_pixels:.0f} px, distance {look.distance_metres:.2f} m: "
            f"tunnel centre is {abs(look.left_offset_metres):.2f} m to your {'LEFT' if look.left_offset_metres > 0 else 'RIGHT'}")
    if camera.latest_points_from_camera is not None:
        depth_offset, _, _ = depth_left_offset_metres(camera.latest_points_from_camera, look.distance_metres)
        if depth_offset is not None:
            text += f" (stereo depth: {abs(depth_offset):.2f} m {'LEFT' if depth_offset > 0 else 'RIGHT'})"
    return text
