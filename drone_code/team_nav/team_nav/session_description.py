"""Describes the computer and the software at the moment the recorder starts, so that every
record says exactly where and with what code it was made.

REAL DRONE CODE. The recorder (flight_recorder.py) saves this as session.json in its run folder,
once the first list of topics is known.

What it records
---------------
  - the computer's name (it must be skybeta-16 on the drone), the time, and how long the
    computer has been switched on ("uptime");
  - ROS_DOMAIN_ID, the ROS network number. Programs only hear each other if they share it, and
    it must differ from every other drone at the venue (section 11 of docs/ARCHITECTURE.md);
  - RMW_IMPLEMENTATION, which of ROS's message delivery libraries is in use;
  - the run mode;
  - a fingerprint ("sha256") of every Python file of team_nav that is installed and running,
    and one fingerprint of them all together. Two records with the same overall fingerprint
    ran the same code. The Jetson's clock may be wrong after a power cut, so this is a more
    trustworthy way to tell which code version made a record than the date;
  - the topics seen so far, and where the records folder physically is (records_space.py).

What it does not do
-------------------
It does not record any setting that might be secret: only the few environment variables named
in ENVIRONMENT_VARIABLES_RECORDED, never the whole environment.
"""
import hashlib
import os
import platform
import socket
import sys
import time

# Our own choice. Only these are recorded; the full environment could hold a password.
ENVIRONMENT_VARIABLES_RECORDED = (
    "ROS_DOMAIN_ID", "ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY",
    "TEAM_NAV_RUN_MODE", "TEAM_NAV_STOP_AFTER_STAGE", "TEAM_NAV_RECORDS_FOLDER", "TEAM_NAV_PICTURE_TOPICS",
)

# Our own choice: the kinds of file whose fingerprints we record.
FINGERPRINTED_FILE_ENDINGS = (".py", ".json")


def describe_session(run_mode, topics_seen, records_storage, package_folder):
    """A dictionary describing this computer and software, ready to be saved as JSON."""
    file_fingerprints = fingerprint_files(package_folder)
    return {
        "computer_name": socket.gethostname(),
        "local_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "unix_time_seconds": time.time(),
        "seconds_since_computer_started": _uptime_seconds(),
        "environment": _recorded_environment(),
        "run_mode": run_mode.name,
        "run_mode_sentence": run_mode.describe(),
        "python_version": sys.version.split()[0],
        "operating_system": platform.platform(),
        "process_number": os.getpid(),
        "records_storage": None if records_storage is None else records_storage.explanation,
        "team_nav_folder": package_folder,
        "team_nav_overall_fingerprint": overall_fingerprint(file_fingerprints),
        "team_nav_file_fingerprints": file_fingerprints,
        "topics_seen": topics_seen,
    }


def fingerprint_files(folder):
    """{path inside folder: sha256 of the file} for every Python and JSON file in folder."""
    fingerprints = {}
    for current_folder, folder_names, file_names in os.walk(folder):
        folder_names[:] = sorted(name for name in folder_names if name != "__pycache__")
        for file_name in sorted(file_names):
            if not file_name.endswith(FINGERPRINTED_FILE_ENDINGS):
                continue
            path = os.path.join(current_folder, file_name)
            with open(path, "rb") as file:
                fingerprints[os.path.relpath(path, folder)] = hashlib.sha256(file.read()).hexdigest()
    return fingerprints


def overall_fingerprint(file_fingerprints):
    """One sha256 of all the files together, from their names and fingerprints in order."""
    combined = hashlib.sha256()
    for path in sorted(file_fingerprints):
        combined.update(f"{path} {file_fingerprints[path]}\n".encode("utf-8"))
    return combined.hexdigest()


def _recorded_environment():
    recorded = {}
    for name in ENVIRONMENT_VARIABLES_RECORDED:
        recorded[name] = os.environ.get(name)
    return recorded


def _uptime_seconds():
    try:
        with open("/proc/uptime", encoding="utf-8") as uptime_file:
            return float(uptime_file.read().split()[0])
    except (OSError, ValueError, IndexError):
        return None
