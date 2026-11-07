"""camera_exposure.py with stand-in parameter services (fake_ros.py): it waits for the camera, sends
one setting at a time with the numbers first, and applies the cap again whenever the camera loses
it."""
import importlib
import sys
import types

import pytest

import fake_ros

LIMIT_MICROSECONDS = 8000
EXPECTED_ORDER = [
    ("left.r_auto_exposure_limit", 8000), ("right.r_auto_exposure_limit", 8000),
    ("left.r_set_auto_exposure_limit", True), ("right.r_set_auto_exposure_limit", True),
    ("left.r_set_man_exposure", False), ("right.r_set_man_exposure", False),
]
BOOL, INTEGER = 1, 2


@pytest.fixture
def make_cap():
    fake_ros.install()
    sys.modules.pop("team_nav.camera_exposure", None)
    camera_exposure = importlib.import_module("team_nav.camera_exposure")

    def make():
        node = fake_ros.Node("test")
        cap = camera_exposure.CameraExposureCap(node, "/oak", LIMIT_MICROSECONDS)
        return cap, CameraServices(node), node
    yield make
    sys.modules.pop("team_nav.camera_exposure", None)
    fake_ros.uninstall()


class CameraServices:
    """The camera's two stand-in parameter services (set and get), handled as one."""

    def __init__(self, node):
        self.node = node
        assert set(node.service_clients) == {"/oak/set_parameters", "/oak/get_parameters"}

    @property
    def requests(self):
        return self.node.service_requests

    def set_ready(self, ready):
        for service_client in self.node.service_clients.values():
            service_client.ready = ready


def accepted():
    return types.SimpleNamespace(results=[types.SimpleNamespace(successful=True, reason="")])


def refused(reason):
    return types.SimpleNamespace(results=[types.SimpleNamespace(successful=False, reason=reason)])


def read_back(values):
    answers = []
    for value in values:
        if isinstance(value, bool):
            answers.append(types.SimpleNamespace(type=BOOL, bool_value=value))
        else:
            answers.append(types.SimpleNamespace(type=INTEGER, integer_value=value))
    return types.SimpleNamespace(values=answers)


def unanswered(client):
    return [request for request in client.requests if request[2].answer is None]


def answer_every_set_in_turn(cap, client):
    """Accept each setting as it is asked for; return the settings in the order they were asked."""
    asked = []
    while unanswered(client):
        assert len(unanswered(client)) == 1                    # never more than one request at a time
        kind, payload, future = unanswered(client)[0]
        assert kind == "set"
        asked += payload
        cap.check()                                            # a check while waiting sends nothing more
        assert len(unanswered(client)) == 1
        future.finish(accepted())
    return asked


def test_waits_for_the_camera_then_sets_one_setting_at_a_time_numbers_first(make_cap):
    cap, client, node = make_cap()
    cap.check()
    assert client.requests == []                               # no camera yet: nothing sent
    client.set_ready(True)
    cap.check()
    assert answer_every_set_in_turn(cap, client) == EXPECTED_ORDER
    assert cap.state == "applied"
    assert any("Camera exposure cap applied on /oak" in text for _, text in node.log_lines)


def test_a_read_back_that_matches_changes_nothing_and_one_that_differs_reapplies(make_cap):
    cap, client, _ = make_cap()
    client.set_ready(True)
    cap.check()
    answer_every_set_in_turn(cap, client)
    cap.check()
    cap.check()                                                # still waiting for the answer: no second read
    assert len(unanswered(client)) == 1
    kind, names, future = unanswered(client)[0]
    assert kind == "get" and names == [name for name, _ in EXPECTED_ORDER]
    future.finish(read_back([value for _, value in EXPECTED_ORDER]))
    assert unanswered(client) == [] and cap.state == "applied"
    cap.check()
    unanswered(client)[0][2].finish(read_back([20000, 20000, False, False, False, False]))   # restarted camera
    assert answer_every_set_in_turn(cap, client) == EXPECTED_ORDER


def test_the_cap_is_applied_again_when_the_camera_comes_back(make_cap):
    cap, client, node = make_cap()
    client.set_ready(True)
    cap.check()
    answer_every_set_in_turn(cap, client)
    client.set_ready(False)
    cap.check()
    assert cap.state == "waiting_for_camera"
    assert any(level == "warn" and "has gone" in text for level, text in node.log_lines)
    client.set_ready(True)
    cap.check()
    assert answer_every_set_in_turn(cap, client) == EXPECTED_ORDER


def test_a_refused_setting_starts_the_whole_sequence_again_at_the_next_check(make_cap):
    cap, client, node = make_cap()
    client.set_ready(True)
    cap.check()
    for _ in range(2):
        unanswered(client)[0][2].finish(accepted())
    unanswered(client)[0][2].finish(refused("parameter not declared"))
    assert unanswered(client) == [] and cap.state == "retry"
    assert any("did not accept left.r_set_auto_exposure_limit = True (parameter not declared)" in text
               for _, text in node.log_lines)
    cap.check()
    assert answer_every_set_in_turn(cap, client) == EXPECTED_ORDER
