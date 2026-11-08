# The box for the full Gazebo simulator, "drone-simulator-gazebo-full". SIMULATOR ONLY.
#
# It starts from the Gazebo trial's box (simulator/gazebo_trial/Dockerfile), which already has
# ROS 2 Humble, PX4 1.16.2 built for Gazebo Harmonic, and Gazebo itself. It adds only OpenCV
# for Python, which the stereo depth check (check_stereo_depth.py) and the team's stereo depth
# code need. The drone's computer has OpenCV too; we have not checked that the versions match.
FROM drone-simulator-gazebo

RUN apt-get update && apt-get install -y --no-install-recommends python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /repository
