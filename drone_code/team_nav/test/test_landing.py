"""landing.py on its own: each step of section 7 of docs/ARCHITECTURE.md, and what it does when the
laser or the position reports misbehave."""
import pytest

from team_nav.landing import Landing
from team_nav.mission_settings import MissionSettings

STEP_SECONDS = 0.1
# Our own choice for these tests: the drone reaches a new commanded speed with about this delay,
# so a hovering drone does not start moving the instant a descent is commanded.
RESPONSE_SECONDS = 0.3
FLOOR_READING_METRES = 0.10
LASER_HEIGHT_ON_FLOOR_METRES = 0.08


class SlowDrone:
    """A drone that follows commands with a short delay. The floor (or a boulder) stops it."""

    def __init__(self, position=(0.0, 0.0, 0.8), floor_height_metres=0.0):
        self.position = list(position)
        self.velocity = [0.0, 0.0, 0.0]
        self.floor_height_metres = floor_height_metres
        self.frozen = False                       # tests set this to imitate motors slow to respond

    def laser_reading(self):
        return max(FLOOR_READING_METRES, self.position[2] - self.floor_height_metres + LASER_HEIGHT_ON_FLOOR_METRES)

    def follow(self, command):
        if command.velocity is not None:
            wanted = list(command.velocity)
        else:
            wanted = [0.95 * (target - now) for target, now in zip(command.hold_position[:3], self.position)]
        if self.frozen:
            wanted = [0.0, 0.0, 0.0]
        for axis in range(3):
            self.velocity[axis] += (wanted[axis] - self.velocity[axis]) * STEP_SECONDS / RESPONSE_SECONDS
            self.position[axis] += self.velocity[axis] * STEP_SECONDS
        if self.position[2] <= self.floor_height_metres:
            self.position[2] = self.floor_height_metres
            self.velocity = [0.0, 0.0, 0.0]


def start_landing(drone, settings=None, sideways_goal_allowed=True):
    landing = Landing(settings or MissionSettings(), FLOOR_READING_METRES, 0.0)
    landing.begin(0.0, (0.0, 0.0, 0.8), (0.0, 0.0, 0.8, 0.0), sideways_goal_allowed)
    return landing


def run(landing, drone, seconds, laser=None, position=None, start_seconds=0.0, every_step=None):
    """laser(drone) and position(drone) replace what the landing hears. Returns
    [(seconds, command, step name, drone height)]."""
    record = []
    now = start_seconds
    while now < start_seconds + seconds - 1e-9:
        if every_step is not None:
            every_step(now, landing, drone)
        heard_laser = drone.laser_reading() if laser is None else laser(drone)
        heard_position = tuple(drone.position) if position is None else position(drone)
        command = landing.step(now, heard_position, heard_laser)
        record.append((now, command, landing.step_name, drone.position[2]))
        drone.follow(command)
        now = round(now + STEP_SECONDS, 9)
    return record


def first(record, step_name):
    return next(entry for entry in record if entry[2] == step_name)


def step_names_in_order(record):
    names = []
    for _, _, name, _ in record:
        if not names or names[-1] != name:
            names.append(name)
    return names


# ---------------------------------------------------------------- the normal landing
def test_follows_the_section_7_steps_in_order():
    drone = SlowDrone()
    record = run(start_landing(drone), drone, 15.0)
    assert step_names_in_order(record) == ["settling", "descending_medium", "descending_gentle", "pushing_down"]
    assert first(record, "descending_medium")[1].velocity[2] == -0.5
    assert first(record, "descending_gentle")[1].velocity[2] == -0.35
    assert first(record, "pushing_down")[1].velocity[2] == -1.0
    gentle_height = first(record, "descending_gentle")[3]
    assert gentle_height == pytest.approx(0.3, abs=0.06)                  # 0.5 m/s until 0.3 m above the floor
    on_floor_at = next(seconds for seconds, _, _, height in record if height == 0.0)
    assert first(record, "pushing_down")[0] - on_floor_at >= 0.3 - 1e-9   # confirmed over 0.3 s


def test_sideways_corrections_steer_over_the_spot_and_stop_below_0_2_metres():
    drone = SlowDrone(position=(0.25, 0.0, 0.8))
    landing = Landing(MissionSettings(), FLOOR_READING_METRES, 0.0)
    landing.begin(0.0, (0.0, 0.0, 0.8), (0.0, 0.0, 0.8, 0.0))             # the spot is 25 cm away
    record = run(landing, drone, 15.0)
    descending = [entry for entry in record if entry[1].velocity is not None and entry[1].velocity[2] < 0]
    assert descending[0][1].velocity[0] < 0.0                             # back toward the spot
    low = [entry for entry in descending if entry[3] < 0.2 - 0.02]
    assert low and all(entry[1].velocity[:2] == (0.0, 0.0) for entry in low)


def test_without_a_sideways_goal_there_are_no_sideways_corrections():
    drone = SlowDrone(position=(0.25, 0.0, 0.8))
    landing = start_landing(drone, sideways_goal_allowed=False)
    record = run(landing, drone, 15.0)
    assert all(entry[1].velocity[:2] == (0.0, 0.0) for entry in record if entry[1].velocity is not None)


class LaserWithTwoBadReadings:
    """Reads the floor for two steps (0.2 s) while the drone is still 0.35 to 0.5 m up."""

    def __init__(self):
        self.bad_readings_given = 0

    def __call__(self, drone):
        if 0.35 < drone.position[2] < 0.5 and self.bad_readings_given < 2:
            self.bad_readings_given += 1
            return FLOOR_READING_METRES
        return drone.laser_reading()


def test_a_floor_reading_shorter_than_0_3_s_does_not_confirm_touchdown():
    drone, laser = SlowDrone(), LaserWithTwoBadReadings()
    record = run(start_landing(drone), drone, 15.0, laser=laser)
    assert laser.bad_readings_given == 2
    assert first(record, "pushing_down")[3] == 0.0                       # only pushed once really on the floor


# ---------------------------------------------------------------- when the laser misbehaves
def test_an_unexpected_laser_reading_holds_to_look_again_then_descends_gently():
    drone = SlowDrone(floor_height_metres=0.53)                            # over a boulder
    record = run(start_landing(drone), drone, 20.0)
    holding = [entry for entry in record if entry[1].reason.startswith("laser reading unexpected")]
    assert holding and all(entry[1].hold_position is not None for entry in holding)
    assert len(holding) * STEP_SECONDS == pytest.approx(3.0, abs=0.35)
    assert "descending_medium" not in step_names_in_order(record)
    assert first(record, "descending_gentle")[1].reason.startswith("laser reads 0.35 m, expected 0.90 m")
    assert step_names_in_order(record)[-1] == "pushing_down" and drone.position[2] == 0.53


def test_a_hovering_drone_with_a_stuck_laser_is_not_a_touchdown():
    def slow_to_respond(now_seconds, landing, drone):
        drone.frozen = landing.step_name == "descending_gentle" and now_seconds - landing.step_start_seconds < 2.0
    drone = SlowDrone()
    record = run(start_landing(drone), drone, 25.0, laser=lambda d: 0.35, every_step=slow_to_respond)
    gentle_start = first(record, "descending_gentle")[0]
    assert all(entry[2] != "pushing_down" for entry in record if entry[0] < gentle_start + 2.0)
    pushed = first(record, "pushing_down")
    assert pushed[3] == 0.0                                                # pushed only once on the floor


def test_a_silent_laser_descends_gently_and_uses_stopping_alone():
    drone = SlowDrone()
    record = run(start_landing(drone), drone, 12.0, laser=lambda d: None)
    assert step_names_in_order(record) == ["settling", "descending_gentle", "pushing_down"]
    assert first(record, "descending_gentle")[1].reason.startswith("laser has no reading")
    assert first(record, "pushing_down")[3] == 0.0


def test_the_laser_is_not_checked_again_after_touchdown():
    def drifting_laser(drone):
        if first_push[0] is None:
            return drone.laser_reading()
        return 0.18                                                        # drifted up on the floor
    first_push = [None]

    def note_push(now_seconds, landing, drone):
        if landing.step_name == "pushing_down" and first_push[0] is None:
            first_push[0] = now_seconds
    drone = SlowDrone()
    record = run(start_landing(drone), drone, 14.0, laser=drifting_laser, every_step=note_push)
    assert first_push[0] is not None
    assert all(entry[2] == "pushing_down" for entry in record if entry[0] >= first_push[0])


# ---------------------------------------------------------------- when position reports stop
def test_without_position_reports_the_laser_alone_confirms_touchdown():
    drone = SlowDrone()
    record = run(start_landing(drone), drone, 20.0, position=lambda d: None)
    assert step_names_in_order(record) == ["settling", "descending_medium", "descending_gentle", "pushing_down"]
    assert first(record, "descending_medium")[0] == pytest.approx(3.0, abs=0.11)   # settling waited its 3 s
    assert first(record, "pushing_down")[3] == 0.0


# ---------------------------------------------------------------- when the motors do not switch off
def test_motors_still_on_keeps_pushing_then_lifts_and_tries_once_more():
    drone = SlowDrone()
    landing = start_landing(drone)
    record = run(landing, drone, 60.0)
    pushes = [entry for entry in record if entry[2] == "pushing_down"]
    first_push_at = pushes[0][0]
    still_on = next(entry for entry in record if entry[1].reason == "motors still on: still pushing down")
    assert still_on[0] - first_push_at == pytest.approx(8.0, abs=0.11)
    lift = first(record, "lifting_for_retry")
    assert lift[0] - first_push_at == pytest.approx(15.0, abs=0.11) and lift[1].velocity[2] == 0.35
    names = step_names_in_order(record)
    assert names[-4:] == ["pushing_down", "lifting_for_retry", "descending_gentle", "pushing_down"]
    assert max(entry[3] for entry in record if entry[0] > lift[0]) >= 0.3 - 0.05   # lifted to about 0.3 m
    assert landing.retries_used == 1 and record[-1][2] == "pushing_down"         # only one retry
