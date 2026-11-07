"""Writes one program's records of one run to disk as it goes, so that a crash or a sudden loss
of power loses at most the last half second.

REAL DRONE CODE. The flight programs and the hardware checks use it. The separate recorder
program (flight_recorder.py) also writes into the records folder, but it keeps its own files.

What it writes
--------------
Each program run gets its own folder inside the records folder, named after the date and time,
the run mode and the program, for example

    ~/team_ws/flight_records/20261108_101502_competition_course_run/

Inside it:
  - events.jsonl: one line per event, such as "route planned", "stage finished" or "stopping
    the run", each a JSON object with the time, a kind, a sentence, and any details. Events are
    rare and important, so each one is forced onto the disk before the program carries on.
  - one .csv file per table of samples, such as the drone's position ten times a second. A row
    is handed to the operating system at once, and forced onto the disk at least every half
    second. Forcing every row would slow the flight loop down for little gain.
  - any whole files the program saves, such as a copy of the planned route.

"Forced onto the disk" means calling os.fsync. Without it, the operating system may hold
written data in memory for many seconds, and a battery pulled out mid-flight would lose it.

Every few seconds it also repeats the run mode in events.jsonl, as section 11 of
docs/ARCHITECTURE.md asks, so that no stretch of a record is ambiguous about it.

What it does not do
-------------------
It does not check free disk space (records_space.py does), it does not subscribe to anything,
and it never raises an error because of a full disk while flying: the flight matters more than
the record. It reports the problem once on the screen and carries on without writing.
"""
import csv
import json
import os
import time

# Our own choice: the environment variable that moves the records folder, for example for tests.
RECORDS_FOLDER_SETTING = "TEAM_NAV_RECORDS_FOLDER"

# Our own choice. ~/team_ws is where the organizers' drone software keeps team code, so it must
# survive the drone's software restarting. records_space.py checks that this is true on the day.
DEFAULT_RECORDS_FOLDER = "~/team_ws/flight_records"

# Our own choice. A power cut loses at most this much of each table of samples.
LONGEST_SECONDS_BEFORE_SAMPLES_ARE_FORCED_TO_DISK = 0.5

# Our own choice. Section 11 asks for the mode to be repeated "every few seconds".
SECONDS_BETWEEN_MODE_REMINDERS = 5.0


def records_folder(environment=None):
    """The folder that holds every run's folder, with ~ expanded."""
    if environment is None:
        environment = os.environ
    return os.path.expanduser(environment.get(RECORDS_FOLDER_SETTING, "") or DEFAULT_RECORDS_FOLDER)


class _Table:
    """One CSV file of samples. The first row decides the columns."""

    def __init__(self, path, column_names):
        self.column_names = list(column_names)
        self.file = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.column_names)
        self.time_last_forced_to_disk = time.monotonic()


class RunLog:

    def __init__(self, program_name, run_mode, folder_holding_all_runs=None):
        """Creates this run's folder and writes the first event, which states the run mode."""
        self.program_name = program_name
        self.run_mode = run_mode
        self.start_monotonic_seconds = time.monotonic()
        self.tables = {}
        self.writing_has_failed = False
        self.time_of_last_mode_reminder = self.start_monotonic_seconds

        if folder_holding_all_runs is None:
            folder_holding_all_runs = records_folder()
        self.folder = self._make_run_folder(folder_holding_all_runs)
        self.events_file = open(os.path.join(self.folder, "events.jsonl"), "a", encoding="utf-8")
        self.event("mode", run_mode.describe(), mode=run_mode.name, stop_after_stage=run_mode.stop_after_stage)
        self.event("records", f"This run's records are in {self.folder}.", folder=self.folder)

    def _make_run_folder(self, folder_holding_all_runs):
        start_time_text = time.strftime("%Y%m%d_%H%M%S")
        base_name = f"{start_time_text}_{self.run_mode.name}_{self.program_name}"
        folder = os.path.join(folder_holding_all_runs, base_name)
        # Two runs started in the same second get separate folders.
        extra_number = 2
        while os.path.exists(folder):
            folder = os.path.join(folder_holding_all_runs, f"{base_name}_{extra_number}")
            extra_number += 1
        os.makedirs(folder)
        return folder

    def seconds_since_start(self):
        return time.monotonic() - self.start_monotonic_seconds

    # ---- Events -------------------------------------------------------------------------------

    def event(self, kind, text, **details):
        """Prints the sentence with the time, and writes it to events.jsonl straight to disk.
        kind is a short label to search for later, such as "stage" or "stop"."""
        print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)
        record = {
            "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "seconds_since_start": round(self.seconds_since_start(), 3),
            "kind": kind,
            "text": text,
        }
        record.update(details)
        self._write_safely(lambda: self._write_event_line(record))

    def _write_event_line(self, record):
        # default=str turns anything JSON can't store, such as a numpy number, into text.
        self.events_file.write(json.dumps(record, default=str) + "\n")
        self.events_file.flush()
        os.fsync(self.events_file.fileno())

    # ---- Samples ------------------------------------------------------------------------------

    def sample(self, table_name, **values):
        """Adds one row to table_name.csv. The first row for a table decides its columns; a later
        row may leave some out (they are written empty) but may not add new ones."""
        self._remind_the_mode_if_due()
        values_with_time = {"seconds_since_start": round(self.seconds_since_start(), 4)}
        values_with_time.update(values)
        self._write_safely(lambda: self._write_sample_row(table_name, values_with_time))

    def _write_sample_row(self, table_name, values):
        table = self.tables.get(table_name)
        if table is None:
            table = _Table(os.path.join(self.folder, f"{table_name}.csv"), values.keys())
            self.tables[table_name] = table
        unknown_columns = [name for name in values if name not in table.column_names]
        if unknown_columns:
            raise ValueError(
                f"The table '{table_name}' was started without the columns {unknown_columns}. "
                f"Give every column in the first sample of a table, even if it is None at first."
            )
        table.writer.writerow([_as_text(values.get(name)) for name in table.column_names])
        table.file.flush()
        now = time.monotonic()
        if now - table.time_last_forced_to_disk >= LONGEST_SECONDS_BEFORE_SAMPLES_ARE_FORCED_TO_DISK:
            os.fsync(table.file.fileno())
            table.time_last_forced_to_disk = now

    def _remind_the_mode_if_due(self):
        if time.monotonic() - self.time_of_last_mode_reminder >= SECONDS_BETWEEN_MODE_REMINDERS:
            self.time_of_last_mode_reminder = time.monotonic()
            self.event("mode", self.run_mode.describe(), mode=self.run_mode.name)

    # ---- Whole files --------------------------------------------------------------------------

    def save_text_file(self, file_name, text):
        """Saves a whole file, such as the planned route, in this run's folder."""
        def write():
            with open(os.path.join(self.folder, file_name), "w", encoding="utf-8") as file:
                file.write(text)
                file.flush()
                os.fsync(file.fileno())
        self._write_safely(write)

    # ---- Closing and errors -------------------------------------------------------------------

    def close(self):
        """Forces everything onto the disk and closes the files. Safe to call more than once."""
        for table in self.tables.values():
            if not table.file.closed:
                self._write_safely(lambda: os.fsync(table.file.fileno()))
                table.file.close()
        if not self.events_file.closed:
            self.events_file.close()

    def _write_safely(self, write):
        """Runs one write. If the disk fails, says so once and stops writing, but never stops
        the program: a lost record is better than a drone that stops being controlled."""
        if self.writing_has_failed:
            return
        try:
            write()
        except OSError as problem:
            self.writing_has_failed = True
            print(f"[{time.strftime('%H:%M:%S')}] RECORDING STOPPED: writing to {self.folder} failed "
                  f"({problem}). The program carries on without recording. Check the free disk space.",
                  flush=True)


def _as_text(value):
    """How one value appears in a CSV cell. Lists and dictionaries are written as JSON."""
    if value is None:
        return ""
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, default=str)
    return value
