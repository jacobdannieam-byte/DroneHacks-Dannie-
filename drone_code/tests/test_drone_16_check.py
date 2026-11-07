"""drone_16_check_on_drone.sh, run on this computer against stand-in docker, ros2 and timeout
commands that play a drone: a healthy one, the one we met on 2026-11-07, and a few single faults.
Test code only. Run from the top of the repository:  python3 -m pytest drone_code/tests
"""
import os
import stat
import subprocess
from pathlib import Path

import pytest

CHECK_SCRIPT = Path(__file__).resolve().parents[1] / "drone_16_check_on_drone.sh"

GOOD_PROGRAMS = """/usr/bin/python3 /opt/ros/humble/bin/ros2 launch team_nav team_nav.launch.py
/usr/bin/python3 /root/skybeta_ws/install/skybeta_flight_interface/lib/skybeta_flight_interface/flight_interface --ros-args
/usr/bin/python3 /root/team_ws/install/team_nav/lib/team_nav/mission --ros-args
/usr/bin/python3 /root/team_ws/install/team_nav/lib/team_nav/corner_count --ros-args
ps -eo args"""
GOOD_PANE = """[mission-2] [INFO] [1.0] [team_nav_mission]: waiting_for_start: ready: waiting for Offboard (the pilot's switch to Position after arming)
[mission-2] [INFO] [6.0] [team_nav_mission]: waiting_for_start: waiting: camera tracking at (0.01, -0.02, 0.00) m, heading 0 deg; 412 corners in the left camera
[corner_count-3] [INFO] [6.1] [team_corner_count]: something else
[mission-2] [INFO] [11.0] [team_nav_mission]: waiting_for_start: waiting: camera tracking at (0.01, -0.02, 0.00) m, heading 0 deg; 409 corners in the left camera"""
GOOD_HZ = """average rate: 29.912
\tmin: 0.021s max: 0.061s std dev: 0.00512s window: 31
average rate: 30.004
\tmin: 0.020s max: 0.332s std dev: 0.00600s window: 151"""
GOOD_BUILD_LOG = "[0.5s] Starting >>> team_nav\n[3.1s] Finished <<< team_nav [2.6s]\n[3.2s] Summary: 1 package finished [3.1s]\n"
GOOD_DRONE = {"version": "0.3.3", "env": "COMPETITOR_PKG=team_nav\nCOMPETITOR_LAUNCH=team_nav.launch.py\n",
              "stack": True, "programs": GOOD_PROGRAMS, "build_log": GOOD_BUILD_LOG, "position": (0.01, -0.02, 0.0),
              "hz": GOOD_HZ, "corners": "412", "state": '{"px4_link": true, "armed": false}', "pane": GOOD_PANE}

STAND_IN_DOCKER = r'''#!/bin/bash
# Plays "docker" on the drone, from the files in $FAKE_DRONE.
if [ "$1" = ps ]; then
    [ -f "$FAKE_DRONE/stack_running" ] && echo skybeta-stack
    exit 0
fi
[ "$1" = exec ] || exit 1
shift 2                                     # "exec skybeta-stack"
case "$1" in
    ps) cat "$FAKE_DRONE/programs" ;;
    tmux) cat "$FAKE_DRONE/pane" ;;
    bash) shift; bash -c "$2" ;;            # "bash -c <script>": run it here, with the stand-in ros2
esac
'''
STAND_IN_ROS2 = r'''#!/bin/bash
# Plays "ros2 topic echo/hz" from the files in $FAKE_DRONE.
case "$*" in
    *"hz /visual_slam/tracking/odometry"*) cat "$FAKE_DRONE/hz" ;;
    *"/visual_slam/tracking/odometry --field pose.pose.position"*) cat "$FAKE_DRONE/position" ;;
    *"/team/left_corner_count"*) cat "$FAKE_DRONE/corners" ;;
    *"/skybeta/state"*) cat "$FAKE_DRONE/state" ;;
esac
'''
STAND_IN_TIMEOUT = '#!/bin/bash\nshift\nexec "$@"\n'


def executable(path, text):
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IEXEC)


def run_check(tmp_path, **changes):
    drone = dict(GOOD_DRONE, **changes)
    fake = tmp_path / "drone"
    tools = tmp_path / "bin"
    for folder in (fake, tools, tmp_path / "skybeta", tmp_path / "team_ws" / "log" / "latest_build"):
        folder.mkdir(parents=True, exist_ok=True)
    if drone["version"] is not None:
        (tmp_path / "skybeta" / "VERSION").write_text(drone["version"] + "\n")
    (tmp_path / "drone.env").write_text(drone["env"])
    if drone["stack"]:
        (fake / "stack_running").write_text("")
    (fake / "programs").write_text(drone["programs"])
    (fake / "pane").write_text(drone["pane"])
    (fake / "hz").write_text(drone["hz"])
    (fake / "corners").write_text(drone["corners"])
    (fake / "state").write_text(drone["state"])
    position = drone["position"]
    (fake / "position").write_text("" if position is None else f"x: {position[0]}\ny: {position[1]}\nz: {position[2]}\n---\n")
    if drone["build_log"] is not None:
        (tmp_path / "team_ws" / "log" / "latest_build" / "logger_all.log").write_text(drone["build_log"])
    executable(tools / "docker", STAND_IN_DOCKER)
    executable(tools / "ros2", STAND_IN_ROS2)
    executable(tools / "timeout", STAND_IN_TIMEOUT)
    environment = dict(os.environ, PATH=f"{tools}:{os.environ['PATH']}", FAKE_DRONE=str(fake),
                       SKYBETA_FOLDER=str(tmp_path / "skybeta"), DRONE_ENV_FILE=str(tmp_path / "drone.env"),
                       TEAM_WORKSPACE=str(tmp_path / "team_ws"))
    finished = subprocess.run(["bash", str(CHECK_SCRIPT), "1.0", "0.3"], capture_output=True, text=True,
                              env=environment, timeout=30)
    return finished.returncode, finished.stdout


def verdict(output):
    return output.strip().splitlines()[-1]


def test_a_healthy_drone_is_ready_to_fly_and_every_line_is_printed(tmp_path):
    code, output = run_check(tmp_path)
    assert code == 0, output
    assert verdict(output) == "READY TO FLY"
    for expected in ("skybeta version:      0.3.3 (at least 0.3.3: yes)",
                     "competitor settings:  COMPETITOR_PKG=team_nav COMPETITOR_LAUNCH=team_nav.launch.py",
                     "programs:             team_nav mission 1, flight_interface 1, square_mission 0, corner_count 1",
                     "last team_ws build:   succeeded",
                     "PX4 link:             up",
                     "odometry position:    (0.01, -0.02, 0.00) m, 0.02 m sideways: inside the start limits",
                     "odometry rate:        30.004 Hz, longest gap 0.332 s (over 5 s)",
                     "left camera corners:  412 (enough to track)",
                     "waiting_for_start: waiting: camera tracking at (0.01, -0.02, 0.00) m, heading 0 deg; 409 corners"):
        assert " ".join(expected.split()) in " ".join(output.split()), expected       # spacing aside
    assert output.count("\n    ") == 3                                   # the mission's last 3 lines


def test_the_drone_we_met_on_2026_11_07_is_not_ready_and_says_why(tmp_path):
    """skybeta 0.3.2, the demo still configured, carried to the net after the stack started."""
    pane = GOOD_PANE.replace("waiting_for_start: ready: waiting for Offboard",
                             "waiting_for_start: not ready: camera tracking timestamps have stopped advancing")
    code, output = run_check(tmp_path, version="0.3.2", position=(10.64, -4.82, -2.41), pane=pane,
                             env="COMPETITOR_PKG=competitor_example\nCOMPETITOR_LAUNCH=competitor.launch.py\n",
                             programs=GOOD_PROGRAMS + "\n/usr/bin/python3 .../square_mission --ros-args")
    assert code == 1
    assert "0.3.2 (at least 0.3.3: NO, PX4 ignores the Offboard request)" in output
    assert "(10.64, -4.82, -2.41) m, 11.68 m sideways: OUTSIDE the start limits" in output
    reasons = verdict(output)
    assert reasons.startswith("NOT READY: skybeta 0.3.2 is older than 0.3.3")
    for reason in ("COMPETITOR_PKG is 'competitor_example'", "square_mission demo is running",
                   "the drone was moved after the stack started", "the mission says not ready: camera tracking"):
        assert reason in reasons, reason


@pytest.mark.parametrize("change, reason", [
    ({"programs": GOOD_PROGRAMS.replace("/lib/team_nav/mission", "/lib/team_nav/other")}, "the team_nav mission is not running"),
    ({"programs": GOOD_PROGRAMS + "\n/x/lib/skybeta_flight_interface/flight_interface"}, "2 flight_interface programs are running"),
    ({"build_log": "Summary: 0 packages finished\n  1 package failed: team_nav\n"}, "the last team_ws build failed (1 package failed: team_nav)"),
    ({"hz": "average rate: 12.000\n\tmin: 0.02s max: 0.700s std dev: 0.1s window: 60"}, "camera tracking is too slow or gappy"),
    ({"hz": ""}, "camera tracking is not reporting"),
    ({"position": None}, "no camera tracking position"),
    ({"position": (0.2, 0.1, 0.45)}, "the drone was moved after the stack started"),
    ({"corners": "7"}, "the left camera sees almost no corners (7)"),
    ({"state": '{"px4_link": false}'}, "flight_interface has no link to PX4"),
    ({"version": None}, "skybeta version unknown"),
    ({"stack": False}, "the stack is not running"),
])
def test_each_fault_alone_makes_it_not_ready_with_its_reason(tmp_path, change, reason):
    code, output = run_check(tmp_path, **change)
    assert code == 1
    assert verdict(output).startswith("NOT READY")
    assert reason in verdict(output), output


def test_a_missing_corner_count_or_build_log_only_warns(tmp_path):
    code, output = run_check(tmp_path, corners="", build_log=None,
                             programs=GOOD_PROGRAMS.replace("/lib/team_nav/corner_count", "/lib/team_nav/x"))
    assert code == 0, output
    assert "warning: no corner count from the left camera" in output
    assert "warning: could not find the last team_ws build log" in output
    assert "warning: corner_count is not running" in output
    assert verdict(output) == "READY TO FLY"


# ---------------------------------------------------------------- the Mac-side scripts
STAND_IN_SSH = r'''#!/bin/bash
# Plays "ssh" and "scp": records each call, answers "hostname" with $FAKE_HOSTNAME.
echo "$*" >> "$SSH_CALLS"
for word in "$@"; do last="$word"; done
[ "$last" = hostname ] && echo "$FAKE_HOSTNAME"
exit 0
'''
SCRIPTS = Path(__file__).resolve().parents[1]


def run_mac_script(tmp_path, script, arguments=(), hostname="skybeta-16", drone_host=None):
    tools = tmp_path / "mac_bin"
    tools.mkdir(exist_ok=True)
    executable(tools / "ssh", STAND_IN_SSH)
    executable(tools / "scp", STAND_IN_SSH)
    calls = tmp_path / "ssh_calls.txt"
    calls.write_text("")
    environment = dict(os.environ, PATH=f"{tools}:{os.environ['PATH']}", SSH_CALLS=str(calls), FAKE_HOSTNAME=hostname)
    environment.pop("DRONE_HOST", None)
    if drone_host is not None:
        environment["DRONE_HOST"] = drone_host
    finished = subprocess.run(["bash", str(SCRIPTS / script), *arguments], capture_output=True, text=True,
                              env=environment, timeout=30)
    return finished.returncode, finished.stdout, calls.read_text().splitlines()


@pytest.mark.parametrize("script", ["install_on_drone_16.sh", "check_drone_16.sh", "restart_drone_16.sh",
                                    "depth_check_16.sh"])
def test_every_script_refuses_a_drone_that_is_not_skybeta_16_before_doing_anything(tmp_path, script):
    if not (SCRIPTS / script).exists():
        pytest.skip(f"{script} is not written yet")
    code, output, calls = run_mac_script(tmp_path, script, ["skybeta-16.local"], hostname="skybeta-7")
    assert code == 1
    assert "not 'skybeta-16'" in output and "nothing has been done" in output
    assert len(calls) == 1 and calls[0].endswith("dronehacks@skybeta-16.local hostname")


@pytest.mark.parametrize("arguments, drone_host, address", [
    (["skybeta-16.local"], None, "skybeta-16.local"),
    ([], "skybeta-16.local", "skybeta-16.local"),
    (["10.0.0.5"], "skybeta-16.local", "10.0.0.5"),
    ([], None, "192.168.55.1"),
])
def test_the_address_comes_from_the_argument_then_drone_host_then_usb(tmp_path, arguments, drone_host, address):
    code, output, calls = run_mac_script(tmp_path, "install_on_drone_16.sh", arguments, drone_host=drone_host)
    assert code == 0, output
    assert all(f"dronehacks@{address}" in call for call in calls)
    assert any("rm -rf ~/team_ws/src/team_nav" in call for call in calls)
    assert any(call.endswith(f"dronehacks@{address}:team_ws/src/team_nav") for call in calls)   # the scp
    assert f"drone_code/restart_drone_16.sh {address}" in output
