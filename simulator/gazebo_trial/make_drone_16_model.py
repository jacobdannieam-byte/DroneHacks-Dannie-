"""Makes a Gazebo drone shaped like drone 16, from PX4's x500 drone. TRIAL ONLY, SIMULATOR ONLY.

Why this file exists
--------------------
PX4 ships one ready-made Gazebo quadcopter, the "x500". It is bigger and heavier than drone 16:
2.0 kg, with its motors 0.25 m from the centre. Our normal simulator uses drone 16's motor
positions and estimated weight and thrust (simulator/px4_changes/drone_16_settings.sh), and PX4
is given those motor positions when it starts. If the Gazebo drone had the x500's shape instead,
PX4 would be steering a drone with its motors in different places from where it thinks they are.

So this script reads PX4's x500 files and writes a copy with drone 16's numbers in it:

  - the four motors where drone 16's motors are;
  - drone 16's estimated weight, 1.47 kg, and the same turning inertia as our normal simulator;
  - propellers 7 inches across, as our referee assumes;
  - a low body instead of the x500's tall legs, so that drone 16's propellers sit 0.10 m above
    the floor when it stands on the ground, as our referee assumes;
  - motor thrust that matches our normal simulator at half and at full power (see below).

The result goes to simulator/gazebo_trial/generated/models/drone_16_like/, with a list of the
PX4 settings that must match it (px4_settings.txt). Gazebo finds the model there through the
GZ_SIM_RESOURCE_PATH setting, and models/drone_16_with_oak_d_s2/ adds our pretend camera to it.
It still uses the x500's pictures (meshes) for its looks, which only matter to a camera, and the
x500's pretend motion, pressure and compass sensors, which PX4 needs.

Run it inside the drone-simulator-gazebo box, which has PX4's files:

    python3 /repository/simulator/gazebo_trial/make_drone_16_model.py

inside_the_box/start_px4_in_gazebo.sh runs it before every flight in Gazebo, so it is always up
to date.

What it does not do
-------------------
It does not make a faithful copy of drone 16. Drone 16's weight, thrust and inertia are
estimates; nobody has weighed it or measured its thrust. Air drag and propeller details are the
x500's.
"""
import os
import xml.etree.ElementTree as ElementTree

PX4_MODELS_FOLDER = "/opt/PX4-Autopilot/Tools/simulation/gz/models"
FOLDER_OF_THIS_FILE = os.path.dirname(os.path.abspath(__file__))
FOLDER_FOR_THE_MODEL = os.path.join(FOLDER_OF_THIS_FILE, "generated", "models", "drone_16_like")
MODEL_NAME = "drone_16_like"

# The same values as simulator/px4_changes/drone_16_settings.sh, which says where each comes from.
# Estimated from the parts list: drone 16 weighs 1.47 kg.
DRONE_MASS_KILOGRAMS = 1.47
# Estimated: each motor gives 7.21 newtons at full power.
FULL_THRUST_PER_MOTOR_NEWTONS = 7.21
# From drone 16's own settings: half power holds the drone up (MPC_THR_HOVER = 0.5).
HOVER_POWER_FRACTION = 0.5

# Where drone 16's motors are, from the organizers' file 07_airframe_geometry.params (the same
# numbers as CA_ROTOR*_PX/PY in drone_16_settings.sh). Here they are in Gazebo's directions:
# forward and LEFT, in metres, so the sign of the sideways number is flipped from PX4's
# forward-and-right. The order is PX4's: front right, back left, front left, back right.
MOTOR_POSITIONS_FORWARD_LEFT_METRES = [
    (0.08, -0.16),
    (-0.15, 0.14),
    (0.08, 0.16),
    (-0.15, -0.14),
]

# From PX4's x500: the propellers spin 6 cm above the centre of the body.
PROPELLER_HEIGHT_ABOVE_CENTRE_METRES = 0.06

# Assumed, as in our referee (referee.py): 7-inch propellers, 0.178 m across.
PROPELLER_DIAMETER_METRES = 0.1778

# From PX4's built-in physics (SIH_IXX, SIH_IYY and SIH_IZZ in sih_params.c), which our normal
# simulator uses because drone 16's real values are unknown. In kilogram square metres.
TURNING_INERTIA_KILOGRAM_SQUARE_METRES = (0.025, 0.025, 0.030)

# Our own choice. The body is a solid box this size, centred on the drone's centre. Its bottom
# is what the drone stands on: 4 cm below the centre, which puts the propellers 6 + 4 = 10 cm
# above the floor, as the referee assumes (PROPELLER_HEIGHT_METRES in referee.py).
BODY_SIZE_METRES = (0.20, 0.20, 0.08)
HEIGHT_OF_CENTRE_ABOVE_FLOOR_WHEN_STANDING_METRES = BODY_SIZE_METRES[2] / 2.0

# How fast a motor spins when PX4 asks for full power, in radians per second. From PX4's x500
# settings (SIM_GZ_EC_MAX = 1000 in its airframe file 4001_gz_x500).
FASTEST_MOTOR_SPEED_RADIANS_PER_SECOND = 1000.0

# Matching our normal simulator's thrust at both half and full power.
# In PX4's built-in physics, thrust grows in a straight line with power, so half power gives
# half of full thrust. A real propeller's thrust grows with the square of its speed, and PX4
# turns power into speed in a straight line, from a slowest speed at zero power to the fastest
# at full power. Half of full thrust then needs 1/sqrt(2) = 0.707 of the fastest speed at half
# power, which happens only if the slowest speed is 2 x 0.707 - 1 = 0.414 of the fastest.
# start_px4_in_gazebo.sh gives PX4 this slowest speed (SIM_GZ_EC_MIN).
SLOWEST_MOTOR_SPEED_RADIANS_PER_SECOND = round(FASTEST_MOTOR_SPEED_RADIANS_PER_SECOND * (2.0 * 0.5 ** 0.5 - 1.0))
# Thrust = MOTOR_CONSTANT x speed squared, so full thrust at the fastest speed fixes it.
MOTOR_CONSTANT = FULL_THRUST_PER_MOTOR_NEWTONS / FASTEST_MOTOR_SPEED_RADIANS_PER_SECOND ** 2

# The x500's collision shapes for its legs, which we remove. The names are PX4's.
X500_LEG_COLLISION_NAMES = [f"base_link_collision_{number}" for number in range(1, 5)]


def make_drone_16_like_model():
    model = _read_x500_body()
    model.set("name", MODEL_NAME)
    # The x500 lifts itself 0.24 m, onto its legs. Ours has no legs: the drone's centre is the
    # model's origin, and the world places it at the right height.
    model.find("pose").text = "0 0 0 0 0 0"
    _make_body_like_drone_16(model)
    _move_motors_and_resize_propellers(model)
    for motor in _read_x500_motors():
        _set_motor_thrust(motor)
        model.append(motor)
    _write_model(model)
    _write_px4_settings_for_this_model()


def _read_x500_body():
    """PX4's x500 comes in two files: x500_base holds the body, propellers and sensors, and
    x500 adds the four motors. This returns the body's <model> element."""
    tree = ElementTree.parse(os.path.join(PX4_MODELS_FOLDER, "x500_base", "model.sdf"))
    return tree.getroot().find("model")


def _read_x500_motors():
    tree = ElementTree.parse(os.path.join(PX4_MODELS_FOLDER, "x500", "model.sdf"))
    return tree.getroot().find("model").findall("plugin")


def _make_body_like_drone_16(model):
    base = model.find("link[@name='base_link']")
    propeller_mass_kilograms = sum(
        float(link.find("inertial/mass").text) for link in model.findall("link") if link.get("name").startswith("rotor_")
    )
    base.find("inertial/mass").text = f"{DRONE_MASS_KILOGRAMS - propeller_mass_kilograms:.4f}"
    inertia = base.find("inertial/inertia")
    for axis_name, value in zip(("ixx", "iyy", "izz"), TURNING_INERTIA_KILOGRAM_SQUARE_METRES):
        inertia.find(axis_name).text = str(value)

    for collision in list(base.findall("collision")):
        if collision.get("name") in X500_LEG_COLLISION_NAMES:
            base.remove(collision)
    body = base.find("collision[@name='base_link_collision_0']")
    body.find("pose").text = "0 0 0 0 0 0"
    body.find("geometry/box/size").text = " ".join(f"{size:.3f}" for size in BODY_SIZE_METRES)


def _move_motors_and_resize_propellers(model):
    for number, (forward_metres, left_metres) in enumerate(MOTOR_POSITIONS_FORWARD_LEFT_METRES):
        rotor = model.find(f"link[@name='rotor_{number}']")
        rotor.find("pose").text = f"{forward_metres} {left_metres} {PROPELLER_HEIGHT_ABOVE_CENTRE_METRES} 0 0 0"
        # The x500's propeller is a thin flat box; we keep its width and thickness.
        size = rotor.find(f"collision[@name='rotor_{number}_collision']/geometry/box/size")
        _, width_metres, thickness_metres = size.text.split()
        size.text = f"{PROPELLER_DIAMETER_METRES} {width_metres} {thickness_metres}"


def _set_motor_thrust(motor):
    motor.find("motorConstant").text = f"{MOTOR_CONSTANT:.6e}"
    motor.find("maxRotVelocity").text = str(FASTEST_MOTOR_SPEED_RADIANS_PER_SECOND)


def _write_model(model):
    os.makedirs(FOLDER_FOR_THE_MODEL, exist_ok=True)
    sdf = ElementTree.Element("sdf", {"version": "1.9"})
    sdf.append(model)
    ElementTree.indent(sdf)
    header = (
        "<?xml version='1.0' encoding='UTF-8'?>\n"
        "<!-- Made by simulator/gazebo_trial/make_drone_16_model.py from PX4's x500. Do not edit. -->\n"
    )
    with open(os.path.join(FOLDER_FOR_THE_MODEL, "model.sdf"), "w", encoding="utf-8") as model_file:
        model_file.write(header + ElementTree.tostring(sdf, encoding="unicode") + "\n")
    with open(os.path.join(FOLDER_FOR_THE_MODEL, "model.config"), "w", encoding="utf-8") as config_file:
        config_file.write(
            f"<?xml version='1.0'?>\n<model>\n  <name>{MODEL_NAME}</name>\n  <version>1.0</version>\n"
            "  <sdf version='1.9'>model.sdf</sdf>\n"
            "  <description>PX4's x500 with drone 16's motor positions, weight and thrust. "
            "Made by make_drone_16_model.py.</description>\n</model>\n"
        )
    print(f"Wrote the drone_16_like model in {os.path.relpath(FOLDER_FOR_THE_MODEL)}")


def _write_px4_settings_for_this_model():
    """Writes the PX4 settings that must match this model, one "NAME VALUE" per line, for
    inside_the_box/start_px4_in_gazebo.sh. SIM_GZ_EC_MIN1 to 4 are the slowest motor speeds
    explained above; PX4's x500 settings would otherwise make them 150 radians per second."""
    path = os.path.join(FOLDER_FOR_THE_MODEL, "px4_settings.txt")
    with open(path, "w", encoding="utf-8") as settings_file:
        for motor_number in range(1, 5):
            settings_file.write(f"SIM_GZ_EC_MIN{motor_number} {SLOWEST_MOTOR_SPEED_RADIANS_PER_SECOND}\n")


if __name__ == "__main__":
    make_drone_16_like_model()
