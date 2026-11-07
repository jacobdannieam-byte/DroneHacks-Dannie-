"""Tests run_mode.py (the competition or practice switch) and run_log.py (records written as the
program goes).

How to run these tests is in the package's README.
"""
import csv
import json
import os

import pytest

from team_nav.run_log import RunLog
from team_nav.run_mode import COMPETITION, PRACTICE, RUN_MODE_SETTING, STOP_AFTER_STAGE_SETTING, read_run_mode


def test_a_missing_setting_means_competition():
    assert read_run_mode({}).name == COMPETITION
    assert read_run_mode({RUN_MODE_SETTING: ""}).name == COMPETITION


def test_practice_mode_can_stop_after_a_stage():
    mode = read_run_mode({RUN_MODE_SETTING: "Practice", STOP_AFTER_STAGE_SETTING: "climb"})

    assert mode.is_practice
    assert mode.stop_after_stage == "climb"
    assert "PRACTICE MODE" in mode.describe()


def test_a_misspelt_mode_is_refused():
    with pytest.raises(ValueError, match="not a mode"):
        read_run_mode({RUN_MODE_SETTING: "practise"})


def test_competition_mode_refuses_to_stop_early():
    with pytest.raises(ValueError, match="whole program must fly"):
        read_run_mode({STOP_AFTER_STAGE_SETTING: "climb"})


def test_the_log_names_its_folder_after_the_mode_and_keeps_events_and_samples(tmp_path):
    log = RunLog("test_program", read_run_mode({RUN_MODE_SETTING: PRACTICE}), str(tmp_path))
    log.event("stage", "Climbing.", target_height_metres=0.8)
    log.sample("position", forward_metres=0.1, left_metres=None, up_metres=[1, 2])
    log.sample("position", forward_metres=0.2)
    log.close()

    assert "_practice_test_program" in os.path.basename(log.folder)
    events = [json.loads(line) for line in open(os.path.join(log.folder, "events.jsonl"))]
    assert events[0]["kind"] == "mode" and events[0]["mode"] == PRACTICE
    assert events[-1]["target_height_metres"] == 0.8
    rows = list(csv.DictReader(open(os.path.join(log.folder, "position.csv"))))
    assert [row["forward_metres"] for row in rows] == ["0.1", "0.2"]
    assert rows[0]["left_metres"] == "" and rows[0]["up_metres"] == "[1, 2]"


def test_a_new_column_after_the_first_row_is_refused(tmp_path):
    log = RunLog("test_program", read_run_mode({}), str(tmp_path))
    log.sample("position", forward_metres=0.1)

    with pytest.raises(ValueError, match="first sample"):
        log.sample("position", forward_metres=0.2, left_metres=0.0)
