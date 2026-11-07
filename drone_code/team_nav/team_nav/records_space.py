"""Checks the disk that holds our records: how much room is left, and whether the records will
still be there after the drone's software is restarted.

REAL DRONE CODE. The recorder (flight_recorder.py) calls this when it starts and every 30
seconds. It only reads; it never deletes anything to make room.

Why "will the records survive?" is a real question
--------------------------------------------------
On the drone, the organizers' software and our code run inside a Docker "container": a sealed
Linux box with its own private filesystem. That private filesystem is thrown away whenever the
box is removed, and drone_code/README.md tells us to remove it ("docker rm -f skybeta-stack")
to restart the software. Only folders that the box borrows from the drone's real disk (a "bind
mount") survive that. The organizers lend the box the team folder, team_ws, so our records
folder must be inside the part of team_ws that is really borrowed.

Linux lists every filesystem a program can see in /proc/self/mountinfo. We find the one that
holds the records folder and look at its type:
  - "overlay" is the container's own private filesystem: records there are lost when the
    container is removed;
  - "tmpfs" or "ramfs" live in memory: records there are lost at power off;
  - anything else (for example "ext4") is a real disk, so records there survive.

What it does not do
-------------------
It does not decide what the recorder records. It only reports the free space and turns it into
one of three recording levels, which flight_recorder.py and recorded_topics.py act on.
"""
import os
import shutil
from dataclasses import dataclass

BYTES_PER_GIGABYTE = 1024 ** 3

# Our own choice. Below this much free space the recorder warns and records less: no camera
# pictures, and at most a few rows per second of each topic. 5 GB is about 10 hours of the
# competition-mode recorder (about 7 MB a minute in the simulator), or about 20 minutes of
# practice mode with camera pictures.
LOW_FREE_SPACE_BYTES = 5 * BYTES_PER_GIGABYTE

# Our own choice. Below this the recorder keeps only a trickle, so that the disk never fills up
# completely. A full disk can break other programs on the drone, including the organizers'.
VERY_LOW_FREE_SPACE_BYTES = 1 * BYTES_PER_GIGABYTE

# The three recording levels. flight_recorder.py and recorded_topics.py use these names.
RECORD_EVERYTHING = "everything"
RECORD_LESS = "less, because the disk is nearly full"
RECORD_A_TRICKLE = "a trickle, because the disk is almost full"

# Filesystem types that do not survive. From the Linux kernel's names for them.
CONTAINER_OWN_FILESYSTEM_TYPES = ("overlay", "aufs")
MEMORY_FILESYSTEM_TYPES = ("tmpfs", "ramfs")


@dataclass(frozen=True)
class RecordsStorage:
    """Where the records folder physically is."""
    mount_point: str          # The folder where that filesystem starts, for example "/root/team_ws".
    filesystem_type: str      # For example "ext4", "overlay" or "tmpfs".
    source: str               # Which disk or which part of it, as Linux names it.
    survives_restart: bool    # False if the records would be lost when the container is removed.
    explanation: str          # One sentence for the screen and the log.


def free_space_bytes(folder):
    """How many bytes are free on the disk that holds folder. If folder does not exist yet, the
    nearest folder above it that does exist is used."""
    existing_folder = _nearest_existing_folder(folder)
    return shutil.disk_usage(existing_folder).free


def recording_level(free_bytes):
    """Turns free space into one of the three recording levels."""
    if free_bytes < VERY_LOW_FREE_SPACE_BYTES:
        return RECORD_A_TRICKLE
    if free_bytes < LOW_FREE_SPACE_BYTES:
        return RECORD_LESS
    return RECORD_EVERYTHING


def describe_free_space(free_bytes):
    """One sentence about the free space, saying what the recorder does about it."""
    free_gigabytes = free_bytes / BYTES_PER_GIGABYTE
    level = recording_level(free_bytes)
    if level == RECORD_EVERYTHING:
        return f"{free_gigabytes:.1f} GB free for records."
    return (f"ONLY {free_gigabytes:.2f} GB FREE for records, so the recorder now records {level}. "
            f"Copy the records to a laptop (drone_code/collect_flight_records.sh) and then delete "
            f"old ones on the drone by hand.")


def find_records_storage(folder, mountinfo_path="/proc/self/mountinfo"):
    """Finds the filesystem that holds folder, and says whether records there survive the
    container being removed. Returns None if mountinfo cannot be read (for example on a Mac)."""
    try:
        with open(mountinfo_path, encoding="utf-8") as mountinfo_file:
            mounts = [_read_mountinfo_line(line) for line in mountinfo_file if line.strip()]
    except OSError:
        return None
    real_folder = os.path.realpath(_nearest_existing_folder(folder))
    mount_point, filesystem_type, source = _longest_matching_mount(real_folder, mounts)
    return RecordsStorage(
        mount_point=mount_point,
        filesystem_type=filesystem_type,
        source=source,
        survives_restart=_survives_restart(filesystem_type),
        explanation=_explain_storage(folder, mount_point, filesystem_type),
    )


def _read_mountinfo_line(line):
    """Reads one line of /proc/self/mountinfo. The format is in "man 5 proc": the fifth field is
    the mount point, then optional fields end with a lone "-", followed by the filesystem type
    and the source."""
    fields = line.split()
    separator_position = fields.index("-")
    mount_point = _undo_octal_escapes(fields[4])
    filesystem_type = fields[separator_position + 1]
    source = fields[separator_position + 2] if len(fields) > separator_position + 2 else ""
    return mount_point, filesystem_type, source


def _undo_octal_escapes(text):
    # mountinfo writes a space in a folder name as "\040". Folder names here rarely have one.
    return text.replace("\\040", " ").replace("\\011", "\t").replace("\\012", "\n").replace("\\134", "\\")


def _longest_matching_mount(real_folder, mounts):
    """The mount whose mount point is the longest start of real_folder. If two filesystems are
    mounted on the same folder, the later one hides the earlier, so the later one wins."""
    best_mount = ("/", "unknown", "")
    for mount_point, filesystem_type, source in mounts:
        inside = real_folder == mount_point or real_folder.startswith(mount_point.rstrip("/") + "/")
        if inside and len(mount_point) >= len(best_mount[0]):
            best_mount = (mount_point, filesystem_type, source)
    return best_mount


def _survives_restart(filesystem_type):
    return filesystem_type not in CONTAINER_OWN_FILESYSTEM_TYPES + MEMORY_FILESYSTEM_TYPES


def _explain_storage(folder, mount_point, filesystem_type):
    if filesystem_type in CONTAINER_OWN_FILESYSTEM_TYPES:
        return (f"WARNING: the records folder {folder} is inside the container's own filesystem "
                f"({filesystem_type}), so every record is LOST when the container is removed, for "
                f"example by 'docker rm -f skybeta-stack'. Copy the records off before restarting "
                f"the drone's software, and set TEAM_NAV_RECORDS_FOLDER to a folder inside the "
                f"borrowed team_ws (see drone_code/COLLECTING_RECORDS.md).")
    if filesystem_type in MEMORY_FILESYSTEM_TYPES:
        return (f"WARNING: the records folder {folder} is in memory ({filesystem_type}), so every "
                f"record is LOST at power off. Set TEAM_NAV_RECORDS_FOLDER to a folder on the disk.")
    return (f"The records folder {folder} is on a real disk ({filesystem_type}, mounted at "
            f"{mount_point}), so the records survive a restart of the drone's software.")


def _nearest_existing_folder(folder):
    folder = os.path.abspath(os.path.expanduser(folder))
    while not os.path.isdir(folder):
        parent = os.path.dirname(folder)
        if parent == folder:
            break
        folder = parent
    return folder
