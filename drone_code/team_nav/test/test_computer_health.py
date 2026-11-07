"""Tests computer_health.py, the recorder's once-a-second measurement of the drone's computer.

Most tests use a made-up /proc folder, so the numbers are known. The last one reads the real
/proc of whatever Linux computer runs the tests. How to run these tests is in the package's
README.
"""
import os
import sys
import time

import pytest

from team_nav.computer_health import ComputerHealth

# Our own choice: long enough for Linux to count about 30 steps of processor time per core.
SECONDS_BETWEEN_REAL_SAMPLES = 0.3

pytestmark = pytest.mark.skipif(not sys.platform.startswith("linux"), reason="reads Linux's /proc")

# A process's /proc/<number>/stat line, with user time 300 ticks, system time 100 ticks and
# 2560 pages of memory. Only those three fields matter here; the name has a space on purpose.
PROCESS_STAT_LINE = ("4242 (python3 x) S 1 4242 4242 0 -1 4194560 100 0 0 0 300 100 0 0 20 10 "
                     "5 0 1000 200000000 2560 18446744073709551615 0 0 0 0 0 0 0 0 0 0 0 0 17 2 0 0 0 0 0")


def make_pretend_computer(tmp_path, core_ticks):
    """A pretend /proc and /sys/class/thermal with two cores, one temperature sensor, our
    recorder, the course program, and two programs that are neither."""
    proc = tmp_path / "proc"
    write_core_ticks(proc, core_ticks)
    (proc / "meminfo").write_text("MemTotal: 7607000 kB\nMemAvailable: 4700000 kB\n"
                                  "SwapTotal: 3804000 kB\nSwapFree: 3804000 kB\n")
    (proc / "loadavg").write_text("1.25 1.10 0.90 2/500 4242\n")
    write_process(proc, "4242", "/usr/bin/python3\0/root/team_ws/install/bin/flight_recorder\0")
    write_process(proc, "77", "/usr/sbin/sshd\0-D\0")
    # A shell that only mentions the recorder in its command must not count as the recorder.
    write_process(proc, "78", "bash\0-c\0ros2 run team_nav flight_recorder\0")
    write_process(proc, "79", "/usr/bin/python3\0-u\0-m\0team_nav.course_run\0")
    thermal = tmp_path / "thermal"
    (thermal / "thermal_zone0").mkdir(parents=True)
    (thermal / "thermal_zone0" / "type").write_text("cpu-thermal\n")
    (thermal / "thermal_zone0" / "temp").write_text("56656\n")
    return str(proc), str(thermal)


def write_core_ticks(proc, core_ticks):
    """core_ticks: (busy, idle) per core."""
    proc.mkdir(exist_ok=True)
    lines = ["cpu  0 0 0 0 0 0 0 0 0 0"]
    for core_number, (busy, idle) in enumerate(core_ticks):
        lines.append(f"cpu{core_number} {busy} 0 0 {idle} 0 0 0 0 0 0")
    (proc / "stat").write_text("\n".join(lines) + "\nintr 0\n")


def write_process(proc, process_number, command):
    folder = proc / process_number
    folder.mkdir()
    (folder / "cmdline").write_bytes(command.encode())
    (folder / "stat").write_text(PROCESS_STAT_LINE + "\n")


def test_processor_memory_temperature_and_programs_are_measured(tmp_path):
    proc, thermal = make_pretend_computer(tmp_path, [(100, 900), (500, 500)])
    health = ComputerHealth(str(tmp_path), proc_folder=proc, thermal_folder=thermal)

    first_row = health.sample()
    # Between the samples, core 0 was busy 50 of 100 ticks and core 1 busy 100 of 100.
    write_core_ticks(tmp_path / "proc", [(150, 950), (600, 500)])
    second_row = health.sample()

    assert list(first_row) == list(second_row), "every row must have the same columns"
    assert first_row["core_0_busy_percent"] is None, "a percentage needs two samples"
    assert second_row["core_0_busy_percent"] == 50.0
    assert second_row["core_1_busy_percent"] == 100.0
    assert second_row["all_cores_busy_percent"] == 75.0
    assert second_row["load_average_1_minute"] == 1.25
    assert second_row["memory_used_megabytes"] == round((7607000 - 4700000) / 1024, 1)
    assert second_row["swap_used_megabytes"] == 0.0
    assert second_row["temperature_cpu_thermal_celsius"] == 56.7
    assert second_row["flight_recorder_processes"] == 1
    assert second_row["flight_recorder_memory_megabytes"] == round(2560 * os.sysconf("SC_PAGE_SIZE") / 1024 / 1024, 1)
    assert second_row["course_run_processes"] == 1, "a module started with python3 -m counts too"
    assert second_row["hardware_check_processes"] == 0
    assert second_row["free_disk_gigabytes"] > 0


def test_the_real_computer_can_be_measured():
    if not os.path.exists("/proc/stat"):
        pytest.skip("this computer has no /proc/stat")
    health = ComputerHealth("/")
    first_row = health.sample()
    # Linux counts processor time in steps of 1/100 s, so the two samples must be further apart
    # than that, or no time has been counted between them.
    time.sleep(SECONDS_BETWEEN_REAL_SAMPLES)
    second_row = health.sample()

    assert list(first_row) == list(second_row)
    assert second_row["all_cores_busy_percent"] is not None
    assert second_row["memory_used_megabytes"] > 0
