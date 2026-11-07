# Scripts that run inside the box

"The box" is the sealed Linux environment that Docker runs on the Mac. These two scripts run inside it, and you normally don't run them yourself, because `../run_test_flight.sh` calls them for you. They are for the simulator only.

1. [`build_our_packages.sh`](build_our_packages.sh) compiles our ROS programs each time the simulator starts.
2. [`load_ros_environment.sh`](load_ros_environment.sh) makes the ROS tools and messages available in the current shell.
