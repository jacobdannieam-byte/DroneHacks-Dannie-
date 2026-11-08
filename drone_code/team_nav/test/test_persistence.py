"""Tests persistence.py: a condition counts only once it has held, without a break, for long
enough. The times are made up; the class never reads a clock itself.

How to run these tests is in the package's README.
"""
from team_nav.persistence import ConditionHeldFor


def test_one_true_reading_is_not_enough():
    held = ConditionHeldFor(3.0)
    assert not held.update(True, 10.0)


def test_a_condition_that_lasts_counts():
    held = ConditionHeldFor(3.0)
    held.update(True, 10.0)
    assert not held.update(True, 12.9)
    assert held.update(True, 13.0)


def test_one_false_reading_restarts_the_clock():
    held = ConditionHeldFor(3.0)
    held.update(True, 10.0)
    held.update(False, 11.0)
    held.update(True, 11.1)
    assert not held.update(True, 13.5)
    assert held.update(True, 14.1)


def test_reset_forgets_the_condition():
    held = ConditionHeldFor(1.0)
    held.update(True, 0.0)
    held.reset()
    assert held.seconds_held(5.0) == 0.0
    assert not held.update(True, 5.0)
