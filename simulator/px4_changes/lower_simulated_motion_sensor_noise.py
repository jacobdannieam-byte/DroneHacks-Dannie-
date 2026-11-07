"""Lowers the noise PX4's physics adds to its pretend motion sensor.

SIMULATOR ONLY. It runs once, while the simulator image is being built, and it edits PX4's
physics code (sih.cpp). The real drone has real sensors, so it never gets this change.

The motion sensor is two parts: an accelerometer, which measures acceleration, and a gyroscope,
which measures how fast the drone is turning.

The problem
-----------
While the motors run, PX4's physics adds strong random noise to these readings, up to 1.7 m/s^2
and 0.14 radians per second. Those values come from a research paper about a fixed-wing plane.
They are about 5 and 9 times what drone 16's settings tell PX4's position estimator to expect.

Normally nobody notices, because simulated GPS keeps correcting the estimate of the drone's
speed. Drone 16 has no GPS. Its only position source is the camera tracking, which reports
position and not speed, so PX4 has to work out speed partly from these noisy readings. With the
original noise, the estimated speed had the wrong sign at times, and the simulated drone wandered
about 0.5 m while trying to hover. That made it fly into walls for a reason that has nothing to do
with the real drone. We saw this on 2026-11-06.

What this does instead
----------------------
It sets the noise to exactly what drone 16's settings say to expect.

This is an estimate. We have not measured the noise on the real drone with its motors running,
and we can't until it has flown. After the first real flight, compare these values with the flight
log. Real vibration may be worse, so a steady hover here doesn't prove a steady hover there.
"""
import sys

# From drone 16's settings: EKF2_ACC_NOISE, in metres per second squared.
ACCELEROMETER_NOISE = "0.35f"
# From drone 16's settings: EKF2_GYR_NOISE, in radians per second.
GYROSCOPE_NOISE = "0.015f"

ORIGINAL_ACCELEROMETER_LINE = "accel_noise = noiseGauss3f(0.5f, 1.7f, 1.4f);"
ORIGINAL_GYROSCOPE_LINE = "gyro_noise = noiseGauss3f(0.14f, 0.07f, 0.03f);"

NEW_ACCELEROMETER_LINE = (
    f"accel_noise = noiseGauss3f({ACCELEROMETER_NOISE}, {ACCELEROMETER_NOISE}, {ACCELEROMETER_NOISE});"
)
NEW_GYROSCOPE_LINE = f"gyro_noise = noiseGauss3f({GYROSCOPE_NOISE}, {GYROSCOPE_NOISE}, {GYROSCOPE_NOISE});"


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

    source_code = replace_line(source_code, ORIGINAL_ACCELEROMETER_LINE, NEW_ACCELEROMETER_LINE)
    source_code = replace_line(source_code, ORIGINAL_GYROSCOPE_LINE, NEW_GYROSCOPE_LINE)

    with open(physics_code_file, "w", encoding="utf-8") as source_file:
        source_file.write(source_code)


if __name__ == "__main__":
    main()
