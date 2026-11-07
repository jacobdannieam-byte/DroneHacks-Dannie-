#!/bin/bash
# Makes the ROS tools and messages available in the current shell. Use it inside the box.
# Each "setup.bash" adds one layer of software, and the order matters.
source /opt/ros/humble/setup.bash                                    # ROS itself
source /opt/px4_messages/install/setup.bash                          # PX4's message definitions
source /repository/simulator/generated_files/ros_install/setup.bash  # Our packages
