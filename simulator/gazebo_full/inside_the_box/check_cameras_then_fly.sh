#!/bin/bash
# Runs inside the full Gazebo simulator's box, once the simulator is starting. SIMULATOR ONLY.
#
# fly_course_in_gazebo.sh runs this with the ROS environment loaded, TEAM_NAV_RUN_MODE and
# TEAM_NAV_RECORDS_FOLDER set, and GAZEBO_FULL_RUN_FOLDER set by the box. Arguments: the longest
# run in seconds, then the arguments for course_run. It:
#   1. starts the team's flight recorder (ros2 run team_nav flight_recorder), as on the drone;
#   2. measures the two camera topics' rates with "ros2 topic hz";
#   3. runs the one-off stereo depth check (check_stereo_depth.py) while the drone stands on
#      the launch mark;
#   4. runs course_run through simulator/test_flights/real_program_test.py, which starts it
#      exactly as "ros2 run team_nav course_run", waits until it sends targets, and then plays
#      the teammate who arms the drone and flips the switch to Position;
#   5. stops the recorder, the camera stamp recorder (record_camera_stamps.py, started before
#      the stereo check) and the team's depth program the way Ctrl+C would, so they close their
#      files.
set -uo pipefail

longest_run_seconds="$1"
shift
run_folder="$GAZEBO_FULL_RUN_FOLDER"

# Python holds back printed text when it goes to a file, and "timeout" stops ros2 topic hz
# before it would write it out. This makes every Python program write each line at once.
export PYTHONUNBUFFERED=1

# Our own choice: long enough for "ros2 topic hz" to start and settle on a steady average. With
# the team's depth program running, 15 s was not enough for it to print anything.
camera_rate_measuring_seconds=30
# Our own choice: how long the recorder gets to close its files after Ctrl+C.
recorder_finishing_seconds=20

echo "[gazebo_full] Starting the team's flight recorder."
ros2 run team_nav flight_recorder > "$run_folder/flight_recorder_output.txt" 2>&1 &
recorder_process_id=$!

echo "[gazebo_full] Waiting for the camera pictures..."
until ros2 topic list 2> /dev/null | grep -q '^/oak/right/image_raw$'; do sleep 2; done
echo "[gazebo_full] Measuring the camera rates for $camera_rate_measuring_seconds seconds with ros2 topic hz."
timeout "$camera_rate_measuring_seconds" ros2 topic hz /oak/left/image_raw > "$run_folder/camera_rate_left.txt" 2>&1 &
left_rate_process_id=$!
timeout "$camera_rate_measuring_seconds" ros2 topic hz /oak/right/image_raw > "$run_folder/camera_rate_right.txt" 2>&1
wait "$left_rate_process_id"
echo "[gazebo_full] Left eye: $(grep 'average rate' "$run_folder/camera_rate_left.txt" | tail -n 1)"
echo "[gazebo_full] Right eye: $(grep 'average rate' "$run_folder/camera_rate_right.txt" | tail -n 1)"

echo "[gazebo_full] Recording every camera picture's time stamp, for the in-flight rates."
python3 /repository/simulator/gazebo_full/record_camera_stamps.py "$run_folder/camera_stamps.csv" > /dev/null 2>&1 &
stamp_recorder_process_id=$!

echo "[gazebo_full] Checking stereo depth once, on the launch mark."
python3 /repository/simulator/gazebo_full/check_stereo_depth.py "$run_folder/stereo_check" \
    || echo "[gazebo_full] The stereo check failed. Its message above says why. The flight goes on."

if [ $# -gt 0 ] && [ "$1" = "ros2" ]; then
    echo "[gazebo_full] Starting flight command: $*"
    python3 /repository/simulator/test_flights/real_program_test.py \
        --longest-run-seconds "$longest_run_seconds" \
        --env "TEAM_NAV_RUN_MODE=$TEAM_NAV_RUN_MODE" \
        --env "TEAM_NAV_RECORDS_FOLDER=$TEAM_NAV_RECORDS_FOLDER" \
        -- "$@"
elif [ $# -gt 0 ] && [ "$1" = "practice_flight" ]; then
    echo "[gazebo_full] Starting practice flight: ros2 run team_nav practice_flight ${*:2}"
    python3 /repository/simulator/test_flights/real_program_test.py \
        --longest-run-seconds "$longest_run_seconds" \
        --env "TEAM_NAV_RUN_MODE=$TEAM_NAV_RUN_MODE" \
        --env "TEAM_NAV_RECORDS_FOLDER=$TEAM_NAV_RECORDS_FOLDER" \
        -- ros2 run team_nav practice_flight "${@:2}"
else
    echo "[gazebo_full] Starting course_run, as on the drone: ros2 run team_nav course_run $*"
    python3 /repository/simulator/test_flights/real_program_test.py \
        --longest-run-seconds "$longest_run_seconds" \
        --env "TEAM_NAV_RUN_MODE=$TEAM_NAV_RUN_MODE" \
        --env "TEAM_NAV_RECORDS_FOLDER=$TEAM_NAV_RECORDS_FOLDER" \
        -- ros2 run team_nav course_run "$@"
fi
flight_exit_code=$?

echo "[gazebo_full] Stopping the flight recorder, the stamp recorder and the team's depth program."
kill -INT "$recorder_process_id" "$stamp_recorder_process_id" 2> /dev/null
# The depth program was started by the launch file, not here. Ctrl+C lets it close its records
# file; otherwise the box is removed under it and the file's last rows are lost.
pkill -INT -f "team_nav/depth_camera" 2> /dev/null
timeout "$recorder_finishing_seconds" tail --pid="$recorder_process_id" -f /dev/null || kill -KILL "$recorder_process_id"
echo "$flight_exit_code" > "$run_folder/flight_program_exit_code.txt"
exit "$flight_exit_code"
