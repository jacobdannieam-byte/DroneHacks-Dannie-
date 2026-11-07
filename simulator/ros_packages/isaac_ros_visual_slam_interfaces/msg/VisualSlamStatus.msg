# A copy of the message the real camera tracking program (cuVSLAM) publishes.
# The field list was read from drone 16 with "ros2 interface show".
#
# vo_state means: 0 = unknown, 1 = tracking fine, 2 = tracking lost.
std_msgs/Header header
uint8 vo_state
float64 node_callback_execution_time
float64 track_execution_time
float64 track_execution_time_mean
float64 track_execution_time_max
