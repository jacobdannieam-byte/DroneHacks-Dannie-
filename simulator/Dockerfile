# Recipe for the box that the simulator runs in.
#
# Why a box at all: the drone's software is built for Linux with ROS 2 Humble, and
# our laptop is a Mac. Docker gives us a small Linux computer inside the Mac, with
# exactly the same software versions as drone 16.
#
# Every version below was read from drone 16 or the organizers' documents.
#
# This file only installs tools. The simulator's own code stays in the repository
# and is shared into the box when it starts, so editing it needs no rebuild.
#
# Build it with: simulator/build_simulator_image.sh  (about 20 minutes, once)

# ROS 2 Humble on Ubuntu 22.04, the same as the drone.
FROM ros:humble-ros-base-jammy

# --- Tools needed to compile PX4, and the Python libraries our simulator uses ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git rsync \
        python3-pip python3-numpy python3-scipy python3-matplotlib python3-empy \
        python3-jinja2 python3-yaml python3-packaging python3-toml python3-future \
        python3-jsonschema python3-serial \
        ros-humble-tf2-ros ros-humble-rosidl-default-generators \
    && pip3 install kconfiglib pyros-genmsg \
    && rm -rf /var/lib/apt/lists/*

# --- The translator between ROS 2 and PX4 -----------------------------------
# PX4 does not speak ROS directly. This small program, called the "agent", passes
# messages between them. Version 2.4.3 is the one the organizers install.
RUN git clone --depth 1 --branch v2.4.3 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/agent \
    && cmake -S /opt/agent -B /opt/agent/build \
    && cmake --build /opt/agent/build --parallel 6 \
    && cmake --install /opt/agent/build \
    && ldconfig \
    && rm -rf /opt/agent

# --- The definitions of the messages PX4 sends and receives -----------------
# This commit is the one installed on drone 16. If the versions did not match,
# messages would be rejected or decoded wrongly.
RUN mkdir -p /opt/px4_messages/src \
    && git clone https://github.com/PX4/px4_msgs.git /opt/px4_messages/src/px4_msgs \
    && git -C /opt/px4_messages/src/px4_msgs checkout 392e831c1f659429ca83902e66820d7094591410 \
    && bash -c "source /opt/ros/humble/setup.bash \
        && cd /opt/px4_messages \
        && colcon build --packages-select px4_msgs \
           --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release"

# --- The real flight control software --------------------------------------
# PX4 version 1.16.2, the release drone 16 runs (the drone has one small extra
# patch from the organizers, for hardware we are not simulating).
RUN git clone --depth 1 --branch v1.16.2 --recurse-submodules --shallow-submodules \
        --jobs 8 https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot

# Our four changes to PX4. Each script explains itself at the top. None of them
# changes how PX4 flies. The first only shares more information, and the other three
# change PX4's physics simulator, which the real drone doesn't use.
COPY px4_changes/let_simulator_read_true_position.py /opt/
RUN python3 /opt/let_simulator_read_true_position.py \
        /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml

COPY px4_changes/lower_simulated_motion_sensor_noise.py /opt/
RUN python3 /opt/lower_simulated_motion_sensor_noise.py \
        /opt/PX4-Autopilot/src/modules/simulation/simulator_sih/sih.cpp

COPY px4_changes/make_simulated_floor_solid.py /opt/
RUN python3 /opt/make_simulated_floor_solid.py \
        /opt/PX4-Autopilot/src/modules/simulation/simulator_sih/sih.cpp

COPY px4_changes/use_drone_16_motor_positions.py /opt/
RUN python3 /opt/use_drone_16_motor_positions.py \
        /opt/PX4-Autopilot/src/modules/simulation/simulator_sih/sih.cpp

# PX4's build stops unless it can read the version label of NuttX. NuttX is the
# operating system of the real flight controller board. Our download above leaves
# out these labels to save time, so we add a placeholder. Its value doesn't matter,
# because the simulator runs on Linux and doesn't use NuttX.
RUN git -C /opt/PX4-Autopilot/platforms/nuttx/NuttX/nuttx tag nuttx-0.0.0

RUN cd /opt/PX4-Autopilot && DONT_RUN=1 make px4_sitl_default

WORKDIR /repository
