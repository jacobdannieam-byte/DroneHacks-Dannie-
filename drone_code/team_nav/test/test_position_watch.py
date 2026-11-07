"""position_watch.py on its own: when repeated positions count as frozen tracking."""
from team_nav.position_watch import PositionWatch

FROZEN_AFTER = 10


def watch():
    return PositionWatch(jump_limit_metres=0.5, step_seconds=0.05, frozen_after_identical_reports=FROZEN_AFTER)


def test_more_than_ten_identical_positions_in_a_row_is_frozen():
    position_watch = watch()
    for report in range(FROZEN_AFTER):
        position_watch.observe(report / 30, (19.5, -6.2, 3.7))
    assert not position_watch.tracking_frozen()                   # ten in a row: not yet
    position_watch.observe(FROZEN_AFTER / 30, (19.5, -6.2, 3.7))
    assert position_watch.tracking_frozen()                       # the eleventh: frozen


def test_any_change_at_all_ends_the_freeze():
    position_watch = watch()
    for report in range(20):
        position_watch.observe(report / 30, (19.5, -6.2, 3.7))
    position_watch.observe(1.0, (19.5, -6.2, 3.7000000000000006))  # the smallest possible change
    assert not position_watch.tracking_frozen()


def test_a_new_run_does_not_clear_a_freeze_because_it_belongs_to_the_live_reports():
    position_watch = watch()
    for report in range(20):
        position_watch.observe(report / 30, (1.0, 2.0, 0.0))
    position_watch.reset()
    assert position_watch.tracking_frozen()
