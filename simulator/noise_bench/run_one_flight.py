"""PRETEND / SIMULATOR ONLY. Never runs on the drone.

Runs ONE bench trial: our real competition program (course_run.py with live navigation), from
the moment it starts until the referee ends the run, against the pretend drone in a pretend
course. Gives back one result row.

    python run_one_flight.py --seed 3 --tracking bad --specks filtered --trace trace.csv

How it fits in
--------------
  1. ros_stand_ins.py makes the real team_nav code importable without ROS.
  2. pretend_world.py builds this trial's true course (the map file, moved a little).
  3. bench_link.BenchNode is the pretend ROS node with the pretend drone behind it.
  4. The real program runs the same steps as course_run.plan_then_fly: plan, make a DroneLink,
     CourseRun(...).run(). It writes its normal records (events.jsonl, live_navigation.csv, ...)
     into the trial's folder.
  5. The referee's result, plus why the program itself stopped, becomes the result row.

What it does not do: run many trials (monte_carlo.py does), or change the real code.
"""
import argparse
import contextlib
import csv
import io
import json
import os
import sys
import tempfile
import time as real_time

import numpy as np

BENCH_FOLDER = os.path.dirname(os.path.abspath(__file__))
REPOSITORY_FOLDER = os.path.dirname(os.path.dirname(BENCH_FOLDER))
TEAM_NAV_FOLDER = os.path.join(REPOSITORY_FOLDER, "drone_code", "team_nav")
sys.path.insert(0, BENCH_FOLDER)
sys.path.insert(0, TEAM_NAV_FOLDER)

from ros_stand_ins import install_ros_stand_ins  # noqa: E402  (must come before team_nav)

install_ros_stand_ins()

from team_nav import course_run, flight_steps  # noqa: E402
from team_nav.run_log import RunLog  # noqa: E402
from team_nav.run_mode import read_run_mode  # noqa: E402

from bench_link import BenchNode, BenchRunIsOver, use_simulated_time  # noqa: E402
from pretend_world import PretendWorld  # noqa: E402

# From course_run.py's defaults: the map and mission the competition run uses.
DEFAULT_MAP_PATH = os.path.join(TEAM_NAV_FOLDER, "course_maps", "sketch_2026-11-08.json")
DEFAULT_MISSION_PATH = os.path.join(TEAM_NAV_FOLDER, "missions", "dronehacks_2026_side_gap_first.json")
# From the task: the scored run is flown in competition mode with live navigation.
RUN_MODE_SETTINGS = {"TEAM_NAV_RUN_MODE": "competition"}
NAVIGATION = "live"

# Event kinds in the program's events.jsonl that say why it stopped (course_run.CourseRun.run).
PROGRAM_STOP_EVENT_KINDS = ["stop", "control_lost", "error", "result"]


def run_one_flight(seed, tracking="clean", specks="filtered", map_path=DEFAULT_MAP_PATH,
                   mission_path=DEFAULT_MISSION_PATH, records_folder=None, trace_path=None):
    """Flies one trial and returns a dictionary of results."""
    random = np.random.default_rng(seed)
    world = PretendWorld(map_path, random)
    trace_rows = [] if trace_path else None
    node = BenchNode(world, random, tracking_starts_bad=(tracking == "bad"), speck_setting=specks,
                     trace_rows=trace_rows)
    records_folder = records_folder or tempfile.mkdtemp(prefix="noise_bench_")
    computer_start = real_time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):
        log, exit_code, bench_stop = _fly_the_real_program(node, map_path, mission_path, records_folder)
    result = {"seed": seed, "tracking": tracking, "specks": specks}
    result.update(node.referee.result())
    result.update(_why_the_program_stopped(log.folder))
    result.update(_live_navigation_summary(log.folder))
    result["exit_code"] = exit_code
    result["bench_stop"] = bench_stop
    liftoff_seconds = node.referee.liftoff_seconds or node.clock.seconds
    result["tracking_frozen_seconds_in_flight"] = round(node.tracking.seconds_frozen(liftoff_seconds, node.clock.seconds), 1)
    result["computer_seconds"] = round(real_time.perf_counter() - computer_start, 1)
    result["records_folder"] = log.folder
    if trace_path:
        _write_trace(trace_path, trace_rows)
    return result


def _fly_the_real_program(node, map_path, mission_path, records_folder):
    """The same steps as course_run.plan_then_fly, with the pretend node instead of ROS."""
    use_simulated_time(node.clock)
    run_mode = read_run_mode(RUN_MODE_SETTINGS)
    log = RunLog(course_run.PROGRAM_NAME, run_mode, folder_holding_all_runs=records_folder)
    arguments = course_run.parse_arguments(["--map", map_path, "--mission", mission_path, "--navigation", NAVIGATION])
    exit_code, bench_stop = None, None
    try:
        plan = course_run.plan_the_course(arguments, run_mode, log)
        link = flight_steps.DroneLink(node, log)
        node.link = link
        exit_code = course_run.CourseRun(plan, run_mode, link, arguments.navigation).run()
    except BenchRunIsOver as stop:
        bench_stop = str(stop)
    finally:
        log.close()
    return log, exit_code, bench_stop


def _why_the_program_stopped(folder):
    """The program's own last word: the last stop, error or result event, cut short."""
    last_text, stage = "", ""
    with open(os.path.join(folder, "events.jsonl"), encoding="utf-8") as events_file:
        for line in events_file:
            event = json.loads(line)
            if event.get("kind") in PROGRAM_STOP_EVENT_KINDS:
                last_text = event["text"]
            if event.get("kind") == "stage" and event.get("event") == "begins":
                stage = event.get("stage", "")
    return {"program_stop": last_text.split("\n")[0][:160], "last_stage": stage}


def _live_navigation_summary(folder):
    """Share of navigation cycles spent in each activity (live_flight.py writes the activity)."""
    path = os.path.join(folder, "live_navigation.csv")
    if not os.path.exists(path):
        return {"navigation_cycles": 0, "top_activities": ""}
    counts = {}
    with open(path, encoding="utf-8") as table:
        for row in csv.DictReader(table):
            counts[row["activity"]] = counts.get(row["activity"], 0) + 1
    total = sum(counts.values())
    ordered = sorted(counts.items(), key=lambda item: -item[1])[:3]
    text = ", ".join(f"{name} {100 * count / total:.0f}%" for name, count in ordered)
    return {"navigation_cycles": total, "top_activities": text}


def _write_trace(path, rows):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as trace_file:
        writer = csv.DictWriter(trace_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description="One noise-bench trial of the real course program (pretend drone).")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--tracking", choices=["clean", "bad"], default="clean")
    parser.add_argument("--specks", choices=["filtered", "old"], default="filtered")
    parser.add_argument("--map", default=DEFAULT_MAP_PATH)
    parser.add_argument("--mission", default=DEFAULT_MISSION_PATH)
    parser.add_argument("--records-folder", default=None, help="Where the program's records go (default: a temp folder).")
    parser.add_argument("--trace", default=None, help="Write a CSV of true positions and targets every 0.1 s here.")
    arguments = parser.parse_args()
    result = run_one_flight(arguments.seed, arguments.tracking, arguments.specks, arguments.map, arguments.mission,
                            arguments.records_folder, arguments.trace)
    for name, value in result.items():
        print(f"{name:28s} {value}")


if __name__ == "__main__":
    main()
