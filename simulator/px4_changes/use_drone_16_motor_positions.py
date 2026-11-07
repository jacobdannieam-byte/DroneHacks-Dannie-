"""Makes PX4's physics turn the drone the way drone 16's real motor positions would.

SIMULATOR ONLY. It runs once, while the simulator image is being built, and it edits PX4's
physics code (sih.cpp). The real drone has real motors in real places, so it never gets this
change.

Why this matters
----------------
A quadcopter tilts by making some motors push harder than others. How much a motor turns the
drone depends on how far it sits from the drone's balance point, just like a longer spanner
turns a bolt more easily.

PX4's physics normally pretends the four motors sit at the corners of a perfect square, 20 cm
from the middle in each direction. Drone 16's frame is not like that. Its front motors are
8 cm in front of the balance point and its back motors are 15 cm behind it. So the back motors
turn the drone more strongly than the front ones, and the front motors must carry more of the
weight to keep the drone level.

PX4's control part already knows the real positions (the CA_ROTOR settings in
drone_16_settings.sh). Without this change, the control part and the physics would disagree
about the frame, which the real drone never has to cope with.

What it still does not copy
---------------------------
  - PX4 measures these positions from the drone's balance point (its centre of mass). The
    organizers entered them, but we don't know whether they measured the real balance point.
    If the battery sits further back, the real drone balances differently.
  - The turning force from each propeller's spin (the yaw part) is still PX4's default.
"""
import sys

# From the organizers' settings file 07_airframe_geometry.params (the CA_ROTOR settings),
# measured on drone 20. The organizers say every drone in the fleet uses the same frame.
# Positions are in metres from the balance point: forward is positive, and right is positive.
MOTOR_FORWARD_POSITIONS_METRES = [0.08, -0.15, 0.08, -0.15]
MOTOR_RIGHT_POSITIONS_METRES = [0.16, -0.14, -0.16, 0.14]

# The lines in sih.cpp that work out how much the motors turn the drone. _u[n] is how hard
# motor n pushes, from 0 to 1, and _T_MAX is its push at full power. The first number in
# Vector3f is the turning force that rolls the drone sideways, the second pitches its nose up,
# and the third turns it left or right. _L_ROLL and _L_PITCH are the 20 cm square.
# We include the line above them, because the same three lines also appear for another kind
# of aircraft further down the file.
ORIGINAL_LINES = (
    "\t\t_T_B = Vector3f(0.0f, 0.0f, -_T_MAX * (+_u[0] + _u[1] + _u[2] + _u[3]));\n"
    "\t\t_Mt_B = Vector3f(_L_ROLL * _T_MAX * (-_u[0] + _u[1] + _u[2] - _u[3]),\n"
    "\t\t\t\t _L_PITCH * _T_MAX * (+_u[0] - _u[1] + _u[2] - _u[3]),\n"
    "\t\t\t\t _Q_MAX * (+_u[0] + _u[1] - _u[2] - _u[3]));\n"
)


def sum_of_motor_pushes_times(distances_metres):
    """Writes C++ code like "(0.08f * _u[0] + -0.15f * _u[1] + ...)".

    Each motor's push, multiplied by its distance from the balance point, gives how much that
    motor turns the drone. Adding them up gives the total.
    """
    terms = []
    for motor_number, distance_metres in enumerate(distances_metres):
        terms.append(f"{distance_metres}f * _u[{motor_number}]")
    return "(" + " + ".join(terms) + ")"


def new_lines():
    # A motor on the right side pushing up rolls the drone towards the left, which PX4 counts as
    # a negative roll, so the right positions get a minus sign. A motor in front pushing up
    # lifts the nose, which PX4 counts as a positive pitch.
    motor_left_positions_metres = []
    for right_position_metres in MOTOR_RIGHT_POSITIONS_METRES:
        motor_left_positions_metres.append(-right_position_metres)

    return (
        "\t\t_T_B = Vector3f(0.0f, 0.0f, -_T_MAX * (+_u[0] + _u[1] + _u[2] + _u[3]));\n"
        "\t\t// Changed for the DroneHacks simulator: drone 16's motor positions, in metres.\n"
        f"\t\t_Mt_B = Vector3f(_T_MAX * {sum_of_motor_pushes_times(motor_left_positions_metres)},\n"
        f"\t\t\t\t _T_MAX * {sum_of_motor_pushes_times(MOTOR_FORWARD_POSITIONS_METRES)},\n"
        "\t\t\t\t _Q_MAX * (+_u[0] + _u[1] - _u[2] - _u[3]));\n"
    )


def main():
    physics_code_file = sys.argv[1]
    with open(physics_code_file, encoding="utf-8") as source_file:
        source_code = source_file.read()

    if source_code.count(ORIGINAL_LINES) != 1:
        raise SystemExit(
            "Could not find the motor turning-force lines exactly once in PX4's physics code. "
            "The PX4 version has probably changed. Read sih.cpp and update this script."
        )
    source_code = source_code.replace(ORIGINAL_LINES, new_lines())

    with open(physics_code_file, "w", encoding="utf-8") as source_file:
        source_file.write(source_code)


if __name__ == "__main__":
    main()
