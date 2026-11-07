"""Table check: the organizers' interface program will not hand the drone to software, and the
organizers' demo mission is not running.

REAL DRONE CODE, run on the table with the PROPELLERS OFF. It only reads.

What it proves
--------------
The organizers' program flight_interface sits between our code and PX4, the flight
controller's software. Two of its settings decide whether software can ever take control:
  - auto_offboard_on_arm: if true, when someone arms the drone and flips the remote's switch to
    Position, the interface hands control to software ("Offboard" mode), which then flies
    whatever program is running. To "arm" a drone is to give its motors permission to spin.
  - allow_mode_request: if true, any program may ask for Offboard mode. The organizers say it is
    for the simulator only.
Our launch file for propellers-off work (launch/watch_only.launch.py) sets both to false, so a
hand test can never turn into a flight. In test_flights.launch.py the hand-over is on, so this
check fails there on purpose. The organizers' demo program
"square_mission" climbs to 1.5 m and flies a 5 m square, which would hit the course walls.

The check reads both settings through flight_interface's parameter service, which only reads,
and lists the running ROS programs. PASS means both settings are false and square_mission is not
running. Once the team adds an approved flight program, auto_offboard_on_arm becomes true on
purpose, and this check then FAILS on purpose: it is for propellers-off work only.
"""
from team_nav.hand_tests.hand_test_tools import failed, passed

# From the organizers' code: the program's ROS name and the two settings (flight_interface.py).
INTERFACE_PROGRAM_NAME = "flight_interface"
SETTINGS_THAT_MUST_BE_FALSE = ["auto_offboard_on_arm", "allow_mode_request"]
# The organizers' demo mission's program name, as the quick first check looks for it
# (drone_code/hardware_checks/on_the_drone/look_at_the_drone.sh). We have not seen its ROS name
# ourselves, so any ROS program whose name contains this counts.
DEMO_PROGRAM_NAME = "square_mission"


def judge_interface_settings(settings, running_program_names):
    """settings maps each setting name to its value, or is None if flight_interface didn't
    answer. running_program_names lists every ROS program's name."""
    if any(DEMO_PROGRAM_NAME in name for name in running_program_names):
        return failed(f"The organizers' demo '{DEMO_PROGRAM_NAME}' is running. It flies a 5 m square as soon as "
                      f"the drone is armed and switched to Position. Replace it (drone_code/README.md) before "
                      f"going further.", running_programs=sorted(running_program_names))
    if settings is None:
        return failed(f"The organizers' {INTERFACE_PROGRAM_NAME} program did not answer, so its settings are unknown. "
                      f"Check it is running ('ros2 node list').", running_programs=sorted(running_program_names))
    switched_on = []
    for name in SETTINGS_THAT_MUST_BE_FALSE:
        if settings.get(name) is not False:
            switched_on.append(f"{name} = {settings.get(name)}")
    if switched_on:
        return failed(f"{INTERFACE_PROGRAM_NAME} has {' and '.join(switched_on)}, so if someone armed the drone and "
                      f"flipped to Position, software would get control. For propellers-off work, start "
                      f"launch/watch_only.launch.py, which sets both to false.", settings=settings)
    return passed(f"{INTERFACE_PROGRAM_NAME} has auto_offboard_on_arm and allow_mode_request both false, and the "
                  f"demo is not running, so no software can take control of the drone.", settings=settings)


# ---- Running it on the drone -------------------------------------------------------------------

def run(listener, person, log):
    running_program_names = listener.node_names()
    settings = None
    if INTERFACE_PROGRAM_NAME in running_program_names:
        settings = listener.read_parameters(INTERFACE_PROGRAM_NAME, SETTINGS_THAT_MUST_BE_FALSE)
    log.event("interface", "The interface's settings and the running programs.", settings=settings,
              running_programs=sorted(running_program_names))
    return judge_interface_settings(settings, running_program_names)
