"""Measures how busy and how hot the drone's main computer (the Jetson) is, once per call.

REAL DRONE CODE. The recorder (flight_recorder.py) calls ComputerHealth.sample() once a second
and writes the result as one row of computer_health.csv. Section 12 of docs/ARCHITECTURE.md asks
how much processor load our software adds while the drone moves; this table answers it.

Where the numbers come from
---------------------------
Everything is read from files that Linux keeps up to date, so nothing extra needs installing:
  /proc/stat                       time each processor core has spent busy and idle, since boot
  /proc/meminfo                    memory in use and available
  /proc/loadavg                    how many programs were waiting to run, averaged over a minute
  /sys/class/thermal/thermal_zone* temperature sensors, in thousandths of a degree Celsius
  /proc/<process number>/stat      time each running program has spent on the processor
  /proc/<process number>/cmdline   the command that started each program, to recognise ours

A processor's "busy percent" can only be measured between two moments, so the first sample
after starting has empty processor columns. 100 percent means one whole core; a program using
two cores fully shows 200 percent.

Inside the organizers' container, /proc/stat and the temperature files describe the whole
Jetson, but the program list only shows programs inside the container. That is where our code
and the organizers' programs run, so it is what we want.

What it does not do
-------------------
It does not write anything, and it does not judge whether a value is too high. It does not
read the graphics processor's load, because we have not checked where drone 16's Jetson
reports it.
"""
import os
import shutil
import time

# From the Linux kernel: /proc times are counted in "clock ticks", and this asks how many ticks
# make one second (100 on almost every Linux computer).
CLOCK_TICKS_PER_SECOND = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100

BYTES_PER_MEGABYTE = 1024 * 1024
BYTES_PER_GIGABYTE = 1024 ** 3

# Our own choice: the programs whose processor and memory use we record, each with a piece of
# text that appears in its program name (see _program_name). The names on the left become
# column names.
#   Our programs, from setup.py's entry points.
#   flight_interface and px4_odom_bridge are the organizers' programs (seen on drone 16 with
#   drone_code/hardware_checks/check_drone_16.sh on 2026-11-07).
#   component_container runs the camera driver and, we believe, the camera tracking (cuVSLAM).
#   MicroXRCEAgent is the program that carries messages between PX4 and ROS.
WATCHED_PROGRAMS = (
    ("course_run", "course_run"),
    ("practice_flight", "practice_flight"),
    ("hardware_check", "hardware_check"),
    ("flight_recorder", "flight_recorder"),
    ("organizers_flight_interface", "flight_interface"),
    ("organizers_position_bridge", "px4_odom_bridge"),
    ("camera_and_tracking_container", "component_container"),
    ("px4_message_link", "MicroXRCEAgent"),
)


class ComputerHealth:

    def __init__(self, folder_for_free_space, proc_folder="/proc", thermal_folder="/sys/class/thermal"):
        self.folder_for_free_space = folder_for_free_space
        self.proc_folder = proc_folder
        self.thermal_folder = thermal_folder
        # The temperature sensors are found once, so that every row has the same columns.
        self.temperature_sensors = self._find_temperature_sensors()
        self.core_count = len(self._read_core_ticks())
        self.previous_core_ticks = None
        self.previous_program_ticks = {}     # Process number -> busy ticks at the last sample.
        self.previous_sample_seconds = None

    def sample(self):
        """One row of measurements, as column name -> value. Every call returns the same columns,
        with None where a value could not be measured."""
        now_seconds = time.monotonic()
        seconds_since_last_sample = None
        if self.previous_sample_seconds is not None:
            seconds_since_last_sample = now_seconds - self.previous_sample_seconds
        self.previous_sample_seconds = now_seconds

        row = {}
        row.update(self._processor_use())
        row.update(self._memory_use())
        row.update(self._temperatures())
        row["free_disk_gigabytes"] = self._free_disk_gigabytes()
        row.update(self._watched_program_use(seconds_since_last_sample))
        return row

    # ---- Processor --------------------------------------------------------------------------

    def _read_core_ticks(self):
        """For each core: (busy ticks, total ticks) since boot, from the "cpuN" lines of /proc/stat."""
        core_ticks = []
        for line in _read_lines(os.path.join(self.proc_folder, "stat")):
            name = line.split(" ", 1)[0]
            if not name.startswith("cpu") or name == "cpu":
                continue
            numbers = [int(number) for number in line.split()[1:]]
            # The fourth and fifth numbers are "idle" and "waiting for the disk".
            idle_ticks = numbers[3] + (numbers[4] if len(numbers) > 4 else 0)
            core_ticks.append((sum(numbers) - idle_ticks, sum(numbers)))
        return core_ticks

    def _processor_use(self):
        core_ticks = self._read_core_ticks()
        row = {"load_average_1_minute": _first_number_in_file(os.path.join(self.proc_folder, "loadavg"))}
        busy_percents = []
        for core_number in range(self.core_count):
            busy_percent = None
            if self.previous_core_ticks is not None and core_number < len(core_ticks):
                busy_percent = _busy_percent(self.previous_core_ticks[core_number], core_ticks[core_number])
            row[f"core_{core_number}_busy_percent"] = busy_percent
            if busy_percent is not None:
                busy_percents.append(busy_percent)
        # The average over all cores: 100 would mean every core fully busy.
        row["all_cores_busy_percent"] = round(sum(busy_percents) / len(busy_percents), 1) if busy_percents else None
        self.previous_core_ticks = core_ticks
        return row

    # ---- Memory and disk --------------------------------------------------------------------

    def _memory_use(self):
        kilobytes = {}
        for line in _read_lines(os.path.join(self.proc_folder, "meminfo")):
            name, _, rest = line.partition(":")
            if rest.split():
                kilobytes[name] = int(rest.split()[0])
        total = kilobytes.get("MemTotal")
        available = kilobytes.get("MemAvailable")
        swap_used = None
        if "SwapTotal" in kilobytes and "SwapFree" in kilobytes:
            swap_used = round((kilobytes["SwapTotal"] - kilobytes["SwapFree"]) / 1024, 1)
        return {
            "memory_used_megabytes": round((total - available) / 1024, 1) if total and available else None,
            "memory_available_megabytes": round(available / 1024, 1) if available else None,
            "swap_used_megabytes": swap_used,
        }

    def _free_disk_gigabytes(self):
        try:
            return round(shutil.disk_usage(self.folder_for_free_space).free / BYTES_PER_GIGABYTE, 2)
        except OSError:
            return None

    # ---- Temperatures -----------------------------------------------------------------------

    def _find_temperature_sensors(self):
        """(column name, file to read) for each temperature sensor, for example
        ("temperature_cpu_thermal_celsius", "/sys/class/thermal/thermal_zone0/temp")."""
        sensors = []
        try:
            zone_names = sorted(os.listdir(self.thermal_folder))
        except OSError:
            return sensors
        for zone_name in zone_names:
            if not zone_name.startswith("thermal_zone"):
                continue
            zone_folder = os.path.join(self.thermal_folder, zone_name)
            sensor_type = _read_text(os.path.join(zone_folder, "type")) or zone_name
            column_name = "temperature_" + _as_column_word(sensor_type) + "_celsius"
            # Two sensors with the same name keep separate columns.
            if column_name in [existing_name for existing_name, _ in sensors]:
                column_name = "temperature_" + _as_column_word(sensor_type + "_" + zone_name) + "_celsius"
            sensors.append((column_name, os.path.join(zone_folder, "temp")))
        return sensors

    def _temperatures(self):
        row = {}
        for column_name, file_path in self.temperature_sensors:
            thousandths_of_a_degree = _first_number_in_file(file_path)
            row[column_name] = None if thousandths_of_a_degree is None else round(thousandths_of_a_degree / 1000, 1)
        return row

    # ---- Our programs -----------------------------------------------------------------------

    def _watched_program_use(self, seconds_since_last_sample):
        """Processor percent and memory of each watched program. Several processes with the same
        name, such as two flight_recorder processes, are added together."""
        totals = {}
        for column_word, _ in WATCHED_PROGRAMS:
            totals[column_word] = {"processes": 0, "busy_percent": None, "memory_megabytes": 0.0}
        new_program_ticks = {}
        for process_number in self._process_numbers():
            column_word = self._watched_program_name(process_number)
            if column_word is None:
                continue
            busy_ticks, memory_bytes = self._read_process_use(process_number)
            if busy_ticks is None:
                continue  # The program ended while we were looking.
            new_program_ticks[process_number] = busy_ticks
            total = totals[column_word]
            total["processes"] += 1
            total["memory_megabytes"] += memory_bytes / BYTES_PER_MEGABYTE
            busy_percent = self._program_busy_percent(process_number, busy_ticks, seconds_since_last_sample)
            if busy_percent is not None:
                total["busy_percent"] = (total["busy_percent"] or 0.0) + busy_percent
        self.previous_program_ticks = new_program_ticks
        return _program_totals_as_columns(totals)

    def _process_numbers(self):
        try:
            names = os.listdir(self.proc_folder)
        except OSError:
            return []
        return [name for name in names if name.isdigit()]

    def _watched_program_name(self, process_number):
        command = _read_text(os.path.join(self.proc_folder, process_number, "cmdline"), keep_nul=True)
        if not command:
            return None
        program = _program_name(command.split("\0"))
        for column_word, text_in_program_name in WATCHED_PROGRAMS:
            if text_in_program_name in program:
                return column_word
        return None

    def _read_process_use(self, process_number):
        """(busy ticks since the program started, memory in bytes), or (None, None)."""
        stat_text = _read_text(os.path.join(self.proc_folder, process_number, "stat"))
        if not stat_text:
            return None, None
        # The program's name sits in brackets and may contain spaces, so we split after it.
        fields_after_name = stat_text.rsplit(")", 1)[1].split()
        # From "man 5 proc": fields 14 and 15 (user and system time) are at positions 11 and 12
        # here, and field 24 (resident memory, in pages) is at position 21.
        busy_ticks = int(fields_after_name[11]) + int(fields_after_name[12])
        memory_bytes = int(fields_after_name[21]) * os.sysconf("SC_PAGE_SIZE")
        return busy_ticks, memory_bytes

    def _program_busy_percent(self, process_number, busy_ticks, seconds_since_last_sample):
        previous_ticks = self.previous_program_ticks.get(process_number)
        if previous_ticks is None or not seconds_since_last_sample:
            return None
        busy_seconds = (busy_ticks - previous_ticks) / CLOCK_TICKS_PER_SECOND
        return 100.0 * busy_seconds / seconds_since_last_sample


def own_processor_seconds():
    """How many seconds of processor time this program has used since it started."""
    times = os.times()
    return times.user + times.system


def _program_name(arguments):
    """The name of the program a process runs, from its start command split into words. For a
    Python program it is the script's name, or the module's after "-m", not "python3". Looking
    only at this name, not the whole command, means a shell or "ros2 run" that merely mentions
    flight_recorder in its command is not counted as the recorder itself."""
    program = os.path.basename(arguments[0])
    if not program.startswith("python"):
        return program
    for position in range(1, len(arguments)):
        argument = arguments[position]
        if argument == "-m" and position + 1 < len(arguments):
            return arguments[position + 1]
        if argument and not argument.startswith("-"):
            return os.path.basename(argument)
    return program


def _program_totals_as_columns(totals):
    row = {}
    for column_word, total in totals.items():
        row[f"{column_word}_processes"] = total["processes"]
        busy_percent = total["busy_percent"]
        row[f"{column_word}_busy_percent"] = None if busy_percent is None else round(busy_percent, 1)
        row[f"{column_word}_memory_megabytes"] = round(total["memory_megabytes"], 1)
    return row


def _busy_percent(earlier_ticks, later_ticks):
    busy_difference = later_ticks[0] - earlier_ticks[0]
    total_difference = later_ticks[1] - earlier_ticks[1]
    if total_difference <= 0:
        return None
    return round(100.0 * busy_difference / total_difference, 1)


def _as_column_word(text):
    """"cpu-thermal" -> "cpu_thermal", so the name is safe as a CSV column."""
    word = ""
    for character in text.strip().lower():
        word += character if character.isalnum() else "_"
    return word


def _read_text(path, keep_nul=False):
    try:
        with open(path, "rb") as file:
            # On drone 16 one temperature file (thermal_zone2) gives no data at all: read()
            # returns None instead of bytes. We treat that as an empty file.
            content = (file.read() or b"").decode("utf-8", errors="replace")
    except OSError:
        return ""
    return content if keep_nul else content.strip()


def _read_lines(path):
    text = _read_text(path)
    return text.splitlines() if text else []


def _first_number_in_file(path):
    text = _read_text(path)
    if not text:
        return None
    try:
        return float(text.split()[0])
    except ValueError:
        return None
