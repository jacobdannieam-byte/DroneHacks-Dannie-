"""Writes a flight's recording: one line per tick, with everything the program knows. FOR THE REAL DRONE.

Each flight gets its own file, named after the program and the start time, in a folder that
survives restarts. On the drone that is ~/team_ws/flight_logs, because the organizers' software
keeps ~/team_ws when the drone's software restarts or updates.

Why it is built this way
------------------------
- Each line is written to the disk straight away, so a crash or a sudden power cut loses at
  most the last line.
- A problem with the recording never stops the flight. If the folder can't be made or a line
  can't be written, it says so once and the flight carries on without recording.
- The file is plain text with columns (CSV), so a spreadsheet or a short Python script can
  read it.
"""
import csv
import os
import time


class TelemetryRecorder:

    def __init__(self, folder, flight_name, say):
        self.say = say
        self.file = None
        self.writer = None
        self.start_seconds = time.monotonic()
        self.problem_reported = False
        try:
            folder = os.path.expanduser(folder)
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, f"{flight_name}_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv")
            self.file = open(path, "w", newline="", encoding="utf-8")
            self.say(f"Recording this flight to {path}.")
        except OSError as problem:
            self.report_problem(problem)

    def write(self, row):
        if self.file is None:
            return
        try:
            full_row = {"seconds_since_program_start": round(time.monotonic() - self.start_seconds, 3)}
            full_row.update(row)
            if self.writer is None:
                self.writer = csv.DictWriter(self.file, fieldnames=list(full_row.keys()))
                self.writer.writeheader()
            self.writer.writerow(full_row)
            self.file.flush()
        except (OSError, ValueError) as problem:
            self.report_problem(problem)

    def report_problem(self, problem):
        if not self.problem_reported:
            self.problem_reported = True
            self.say(f"The flight recording failed ({problem}). The flight carries on without it.")
        self.file = None

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None
