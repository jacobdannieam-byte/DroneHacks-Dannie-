"""PRETEND / SIMULATOR ONLY. Never runs on the drone.

Runs many seeded bench trials of the real course program in parallel and prints a table: how
often it finishes, how far it gets, touches, time, why runs end and where they get stuck.

    python monte_carlo.py --trials 20                 # 20 seeds x 4 conditions
    python monte_carlo.py --trials 20 --tracking bad --specks filtered

The conditions are camera tracking (the trial STARTS in its clean or bad state and may switch,
pretend_tracking.py) and false depth specks
(filtered, today's filter, or old, before it; pretend_world.py). Seed n is the same pretend
course and the same noise in every run of this script, so to judge a change to a constant,
change it, run the same seeds again and compare the two tables (README.md says more).

How it fits in: each trial is run_one_flight.run_one_flight in its own process.
What it does not do: decide whether the program is good enough. People do.
"""
import argparse
import multiprocessing
import os
import time

import pandas as pd

from run_one_flight import run_one_flight

# Our own choice: leave one core for the rest of the Mac.
WORKER_COUNT = max(1, (os.cpu_count() or 2) - 1)
# Our own choice: where a stuck run ended, to the nearest this, so similar spots group together.
WHERE_ROUNDING_METRES = 0.5


def run_trial(settings):
    seed, tracking, specks = settings
    try:
        return run_one_flight(seed, tracking, specks)
    except Exception as problem:     # A bench bug in one trial must not lose the other results.
        return {"seed": seed, "tracking": tracking, "specks": specks, "referee_end": f"BENCH ERROR: {problem}"}


def summary_table(results):
    rows = []
    for (tracking, specks), group in results.groupby(["tracking", "specks"]):
        finished = group[group["finished"] == True]   # noqa: E712  (pandas needs ==)
        rows.append({
            "tracking": tracking, "specks": specks, "trials": len(group),
            "finish rate": f"{100 * len(finished) / len(group):.0f}%",
            "program says finished": f"{100 * group['program_stop'].fillna('').str.startswith('RUN FINISHED').mean():.0f}%",
            "mean checkpoints": round(group["checkpoints"].mean(), 2),
            "mean lines passed (any order)": round(group["lines_passed_any_order"].mean(), 2),
            "mean touches": round(group["touches"].mean(), 2),
            "mean flight s": round(group["flight_seconds"].mean(), 1),
            "mean finish s": round(finished["flight_seconds"].mean(), 1) if len(finished) else None,
            "mean score": round(group["score"].mean(), 1),
        })
    return pd.DataFrame(rows)


def counted(results, column):
    return results.groupby(["tracking", "specks"])[column].value_counts().rename("trials").reset_index()


def where_runs_ended(results):
    stuck = results[results["finished"] != True].copy()   # noqa: E712
    stuck["where"] = [f"({round(east / WHERE_ROUNDING_METRES) * WHERE_ROUNDING_METRES:+.1f}, "
                      f"{round(north / WHERE_ROUNDING_METRES) * WHERE_ROUNDING_METRES:+.1f})"
                      for east, north in zip(stuck["end_east_metres"].fillna(0), stuck["end_north_metres"].fillna(0))]
    return stuck.groupby(["last_stage", "where"]).size().rename("trials").reset_index().sort_values("trials", ascending=False)


def main():
    parser = argparse.ArgumentParser(description="Many noise-bench trials of the real course program (pretend drone).")
    parser.add_argument("--trials", type=int, default=20, help="Seeds per condition.")
    parser.add_argument("--first-seed", type=int, default=1)
    parser.add_argument("--tracking", nargs="*", default=["clean", "bad"])
    parser.add_argument("--specks", nargs="*", default=["filtered", "old"])
    parser.add_argument("--output", default="monte_carlo_results.csv")
    arguments = parser.parse_args()
    settings = [(seed, tracking, specks)
                for tracking in arguments.tracking for specks in arguments.specks
                for seed in range(arguments.first_seed, arguments.first_seed + arguments.trials)]
    start = time.perf_counter()
    with multiprocessing.Pool(WORKER_COUNT) as pool:
        results = pd.DataFrame(pool.map(run_trial, settings, chunksize=1))
    results.to_csv(arguments.output, index=False)
    pd.set_option("display.width", 250)
    pd.set_option("display.max_colwidth", 120)
    print(f"{len(settings)} trials in {time.perf_counter() - start:.0f} s. Every row: {arguments.output}\n")
    print(summary_table(results).to_string(index=False), "\n")
    print("Why the referee ended each run:")
    print(counted(results, "referee_end").to_string(index=False), "\n")
    print("What the program itself said last:")
    print(counted(results, "program_stop").to_string(index=False), "\n")
    print("Where unfinished runs ended (stage, rounded east/north):")
    print(where_runs_ended(results).to_string(index=False))


if __name__ == "__main__":
    main()
