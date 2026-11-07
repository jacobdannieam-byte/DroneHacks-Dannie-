"""Tests records_space.py: the free space check, and whether the records folder survives the
drone's software being restarted.

The mount tables here are made up, in the format of Linux's /proc/self/mountinfo, so the tests
run anywhere. How to run these tests is in the package's README.
"""
import os

from team_nav.records_space import (
    BYTES_PER_GIGABYTE, RECORD_A_TRICKLE, RECORD_EVERYTHING, RECORD_LESS, describe_free_space,
    find_records_storage, free_space_bytes, recording_level,
)


def write_mount_table(tmp_path, lines):
    mountinfo_path = tmp_path / "mountinfo"
    mountinfo_path.write_text("\n".join(lines) + "\n")
    return str(mountinfo_path)


def container_mount_table(team_folder):
    """What a Docker container typically sees: its own overlay filesystem at the top, and the
    team folder borrowed from the real disk (a "bind mount")."""
    return [
        "800 700 0:52 / / rw,relatime master:1 - overlay overlay rw,lowerdir=/var/lib/docker/a,upperdir=/var/lib/docker/b",
        "801 800 0:55 / /proc rw,nosuid,nodev,noexec,relatime - proc proc rw",
        "802 800 0:56 / /dev/shm rw,nosuid,nodev - tmpfs shm rw,size=65536k",
        f"803 800 259:2 /home/dronehacks/team_ws {team_folder} rw,relatime - ext4 /dev/nvme0n1p1 rw",
    ]


def test_records_inside_the_borrowed_team_folder_survive(tmp_path):
    team_folder = os.path.realpath(tmp_path / "team_ws")
    records = os.path.join(team_folder, "flight_records")
    os.makedirs(records)
    mountinfo = write_mount_table(tmp_path, container_mount_table(team_folder))

    storage = find_records_storage(records, mountinfo)

    assert storage.survives_restart
    assert storage.filesystem_type == "ext4"
    assert storage.mount_point == team_folder
    assert "survive" in storage.explanation


def test_records_in_the_containers_own_filesystem_are_flagged(tmp_path):
    team_folder = os.path.realpath(tmp_path / "team_ws")
    records_outside_the_team_folder = os.path.realpath(tmp_path / "somewhere_else")
    os.makedirs(records_outside_the_team_folder)
    mountinfo = write_mount_table(tmp_path, container_mount_table(team_folder))

    storage = find_records_storage(records_outside_the_team_folder, mountinfo)

    assert not storage.survives_restart
    assert storage.filesystem_type == "overlay"
    assert "LOST" in storage.explanation and "docker rm" in storage.explanation


def test_records_in_memory_are_flagged(tmp_path):
    memory_folder = os.path.realpath(tmp_path / "in_memory")
    os.makedirs(memory_folder)
    mountinfo = write_mount_table(tmp_path, [
        "1 0 0:1 / / rw - ext4 /dev/sda1 rw",
        f"2 1 0:2 / {memory_folder} rw - tmpfs tmpfs rw",
    ])

    storage = find_records_storage(os.path.join(memory_folder, "not_made_yet"), mountinfo)

    assert not storage.survives_restart
    assert "power off" in storage.explanation


def test_a_missing_mount_table_gives_no_answer_rather_than_an_error(tmp_path):
    assert find_records_storage(str(tmp_path), str(tmp_path / "no_such_file")) is None


def test_free_space_works_for_a_folder_not_made_yet(tmp_path):
    assert free_space_bytes(str(tmp_path / "not" / "made" / "yet")) > 0


def test_less_free_space_means_less_recording():
    assert recording_level(100 * BYTES_PER_GIGABYTE) == RECORD_EVERYTHING
    assert recording_level(3 * BYTES_PER_GIGABYTE) == RECORD_LESS
    assert recording_level(0.5 * BYTES_PER_GIGABYTE) == RECORD_A_TRICKLE
    assert "ONLY" in describe_free_space(3 * BYTES_PER_GIGABYTE)
    assert "ONLY" not in describe_free_space(100 * BYTES_PER_GIGABYTE)
