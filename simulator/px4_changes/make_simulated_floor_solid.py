"""Makes the floor in PX4's physics behave like a real, solid floor.

SIMULATOR ONLY. It runs once, while the simulator image is being built, and it edits PX4's
physics code (sih.cpp). The real drone stands on a real floor, so it never gets this change.

It fixes two things that matter because we use this simulator to study landing.

1. The drone could sink through the floor
-----------------------------------------
PX4's physics only stops the drone at floor level if the total force on it points downward.
While landing, the motors are still holding up almost the whole weight, so the total force is
close to zero and is sometimes slightly upward. The drone then kept moving down, and it sank
13 cm below the floor before stopping (seen on 2026-11-06). Now the floor also stops a drone that
is moving downward, whatever the forces are.

2. A drone on the floor could tilt freely
-----------------------------------------
PX4's physics keeps letting the drone turn while it is on the floor, so it could lean a few
degrees before takeoff. Real landing legs prevent that. Now a drone on the floor can't turn.

What it still does not copy: the legs have no springiness, so the drone stops dead in one physics
step and does not bounce.
"""
import sys

ORIGINAL_FLOOR_CHECK = "if ((_lla.altitude() - _lpos_ref_alt) < 0.f && force_down > 0.f) {"
# _v_N(2) is the drone's speed downward, in metres per second. We only count speeds above 1 cm/s,
# which is our own choice. A standing drone has a tiny speed left over from rounding errors. If that
# counted as moving down, the floor would hold the drone forever and it could never take off.
NEW_FLOOR_CHECK = "if ((_lla.altitude() - _lpos_ref_alt) < 0.f && (force_down > 0.f || _v_N(2) > 0.01f)) {"

ORIGINAL_TURNING_LINE = "_w_B = constrain(_w_B + w_B_dot * dt, -6.0f * M_PI_F, 6.0f * M_PI_F);"
# _w_B is how fast the drone is turning.
NEW_TURNING_LINES = ORIGINAL_TURNING_LINE + "\n\n\tif (_grounded) { _w_B.setZero(); }"


def replace_line(source_code, original_line, new_line):
    if source_code.count(original_line) != 1:
        raise SystemExit(
            f"Could not find this line exactly once in PX4's physics code: {original_line} "
            "The PX4 version has probably changed. Read sih.cpp and update this script."
        )
    return source_code.replace(original_line, new_line)


def main():
    physics_code_file = sys.argv[1]
    with open(physics_code_file, encoding="utf-8") as source_file:
        source_code = source_file.read()

    source_code = replace_line(source_code, ORIGINAL_FLOOR_CHECK, NEW_FLOOR_CHECK)
    source_code = replace_line(source_code, ORIGINAL_TURNING_LINE, NEW_TURNING_LINES)

    with open(physics_code_file, "w", encoding="utf-8") as source_file:
        source_file.write(source_code)


if __name__ == "__main__":
    main()
