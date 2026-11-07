#!/bin/bash
# Starts the real PX4 flight software, using its built-in physics. SIMULATOR ONLY.
#
# "SIH" means Simulation In Hardware. It is a small physics model inside PX4 that
# works out how a quadcopter moves. It needs no 3D graphics, which is why it can run
# on our Mac. It knows about gravity, thrust and a flat floor, but not about walls.
set -e

folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"
source "$folder_of_this_script/drone_16_settings.sh"

# 10040 is PX4's number for its simulated X-shaped quadcopter.
export PX4_SYS_AUTOSTART=10040
export PX4_SIMULATOR=sihsim
export PX4_SIM_MODEL=sihsim_quadx

# Makes PX4's simulated clock keep up with real time. PX4 does one physics step every 4
# milliseconds and sleeps in between. Inside the Linux box on a Mac, every sleep runs over
# by about 1 to 1.6 milliseconds, so at the normal setting of 1.0 PX4 runs at only 73% of
# real time, and everything would happen in slow motion compared with the pretend sensors.
#
# Measured on an M1 Mac on 2026-11-06. With PX4 running alone, the result grows in step with
# this setting: 1.00 gave 0.73 of real time, 1.15 gave 0.83, and 1.30 gave 0.95. With the whole
# simulator running, the sleeps are more accurate, and 1.30 gave 1.11. So we use 1.17.
# The referee measures the clock speed during each flight and puts it in the flight's summary.
#
# Because this setting is present, PX4's start-up script tries to adjust four timeout settings
# using a calculator program called bc, which isn't installed. It then prints "not enough
# arguments" four times and leaves those settings as they were. That is what we want.
export PX4_SIM_SPEED_FACTOR=1.17

px4_build_folder=/opt/PX4-Autopilot/build/px4_sitl_default
mkdir -p "$px4_build_folder/rootfs"
cd "$px4_build_folder/rootfs"

# PX4 saves its settings here. We delete them, so every run starts from the same
# known state and nothing is carried over from the last run.
rm -f parameters.bson parameters_backup.bson

# PX4 writes a detailed record of each flight into a folder called "log". We keep those
# records in the repository's generated files, because the box is deleted after each flight.
mkdir -p /repository/simulator/generated_files/px4_flight_logs
ln -sfn /repository/simulator/generated_files/px4_flight_logs log

# -d means "no interactive prompt". The next word says where PX4's start-up files are,
# and -s names the one to run, relative to that folder.
exec ../bin/px4 -d "$px4_build_folder/etc" -s etc/init.d-posix/rcS
