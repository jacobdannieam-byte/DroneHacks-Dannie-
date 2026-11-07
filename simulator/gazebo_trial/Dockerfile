# Recipe for the box that the Gazebo trial runs in. TRIAL ONLY, SIMULATOR ONLY.
#
# What this adds: Gazebo, a robot simulator that can draw camera and depth
# pictures, on top of our normal simulator box ("drone-simulator", made by
# simulator/Dockerfile). PX4, ROS 2 Humble, px4_msgs and the message agent all
# come from that box unchanged, so the versions still match drone 16.
#
# What this does not change: the "drone-simulator" box itself. This recipe only
# adds layers on top of it and gives the result its own name,
# "drone-simulator-gazebo". Deleting this box leaves the normal simulator as it was.
#
# Build it with: simulator/gazebo_trial/build_gazebo_image.sh
# (the normal simulator box must be built first)

FROM drone-simulator

# --- Gazebo Harmonic ---------------------------------------------------------
# PX4 version 1.16 needs Gazebo "Harmonic" (version 8 of its simulator part).
# Ubuntu does not ship it, so we add Gazebo's own package list. Its packages
# are built for the Mac's arm64 processor as well as for normal PCs.
#
# Other packages here:
#   libopencv-*-dev   PX4's Gazebo add-ons need OpenCV to compile.
#   mesa, egl, xvfb   Software drawing. The box has no graphics card, so the
#                     camera pictures are drawn by the processor (Mesa's
#                     "llvmpipe"). EGL lets Gazebo draw without a screen; Xvfb
#                     is a pretend screen, kept as a fallback.
RUN apt-get update && apt-get install -y --no-install-recommends curl lsb-release gnupg \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        libopencv-core-dev libopencv-imgproc-dev \
        libgl1-mesa-dri libegl1 libegl-mesa0 libgles2 mesa-utils xvfb \
    && rm -rf /var/lib/apt/lists/*

# --- PX4 again, this time with its Gazebo connection --------------------------
# PX4 only builds its link to Gazebo (a part called "gz_bridge") if it finds
# Gazebo while compiling. The normal box compiled PX4 before Gazebo existed, so
# we compile it again here. We delete the old build folder first, because PX4's
# build remembers "Gazebo not found" from last time.
#
# Note: the PX4 source here still carries our four edits from px4_changes/.
# Three of them only change PX4's own simple physics (SIH), which Gazebo
# replaces, so they have no effect in this box.
RUN cd /opt/PX4-Autopilot \
    && rm -rf build/px4_sitl_default \
    && GZ_DISTRO=harmonic DONT_RUN=1 make px4_sitl_default

WORKDIR /repository
