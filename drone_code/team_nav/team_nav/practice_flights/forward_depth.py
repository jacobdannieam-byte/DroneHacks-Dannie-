"""Hold translation when stereo depth is stale or shows an obstacle in the next short step.

Uses stamped ROS depth points and the matching tracked camera pose, not simulator truth.
It does not replan or claim to see behind the aircraft during the short mapped retreat.
"""
import math
import numpy as np
from team_nav.depth_camera import OBSTACLE_POINTS_TOPIC, points_from_message
from team_nav.flight_steps import rotation_matrix, heading_from_rotation
from team_nav.practice_flights.five_waypoint_plan import angle_difference

# Our own choices: arrival and exposure must both be recent; four close points reject speckles.
MAXIMUM_DEPTH_AGE_SECONDS = 1.5
MINIMUM_OBSTACLE_POINTS = 4
FORWARD_BODY_AND_MARGIN_METRES = .22
SIDE_HALF_WIDTH_METRES = .29
VERTICAL_HALF_HEIGHT_METRES = .15
MAXIMUM_LOOKAHEAD_METRES = .45
MAXIMUM_PICTURE_HEADING_ERROR_RADIANS = math.radians(8)


class ForwardDepth:
    def __init__(self, link, frame):
        from sensor_msgs.msg import PointCloud2
        self.link, self.frame = link, frame
        self.arrival = None
        self.stamp = None
        self.points = np.empty((0,3))
        self.messages_used = 0
        self.picture_heading = None
        link.node.create_subscription(PointCloud2, OBSTACLE_POINTS_TOPIC, self.receive, 2)

    def receive(self, message):
        if self.link.launch is None:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        exposure_age = self.link.node.get_clock().now().nanoseconds*1e-9-stamp
        if not -.1 <= exposure_age <= MAXIMUM_DEPTH_AGE_SECONDS:
            return
        pose = self.link.tracked_pose_at(stamp)
        if pose is None or (self.stamp is not None and stamp <= self.stamp):
            return
        position, quaternion = pose
        points = points_from_message(message)
        points = points[np.all(np.isfinite(points),axis=1)]
        if not len(points):
            return  # No matches is unknown space, not proof that the path is clear.
        odom = points @ rotation_matrix(quaternion).T + position - self.link.launch.tracked_position_metres
        xy = (self.frame.launch_mark_centre_metres + odom[:,0:1]*self.frame.forward_on_course
              + odom[:,1:2]*self.frame.left_on_course + self.frame.drift_correction_metres)
        self.points = np.column_stack((xy, odom[:,2]))
        self.picture_heading = self.frame.course_heading_from_start_frame(heading_from_rotation(quaternion))
        self.arrival, self.stamp = self.link.now_seconds(), stamp
        self.messages_used += 1

    def reason_to_hold(self, position, goal, heading, reverse=False):
        if self.arrival is None:
            return 'waiting for the first usable depth picture'
        exposure_age = self.link.node.get_clock().now().nanoseconds*1e-9-self.stamp
        if self.link.now_seconds()-self.arrival > MAXIMUM_DEPTH_AGE_SECONDS or not -.1 <= exposure_age <= MAXIMUM_DEPTH_AGE_SECONDS:
            return 'depth picture is stale'
        if self.picture_heading is None or abs(angle_difference(heading, self.picture_heading)) > MAXIMUM_PICTURE_HEADING_ERROR_RADIANS:
            return 'waiting for depth facing the new direction'
        if reverse:
            return None  # Only the separately validated 0.4 m retrace uses this exception.
        return obstacle_reason(self.points, position, goal, heading)


def obstacle_reason(points, position, goal, heading):
    offset = points-position
    forward = offset[:,0]*math.cos(heading)+offset[:,1]*math.sin(heading)
    left = -offset[:,0]*math.sin(heading)+offset[:,1]*math.cos(heading)
    ahead = min(MAXIMUM_LOOKAHEAD_METRES, np.linalg.norm(goal[:2]-position[:2])+FORWARD_BODY_AND_MARGIN_METRES)
    close = ((forward > 0) & (forward < ahead) & (np.abs(left)<SIDE_HALF_WIDTH_METRES)
             & (np.abs(offset[:,2])<VERTICAL_HALF_HEIGHT_METRES))
    return 'obstacle in the next forward step' if np.count_nonzero(close)>=MINIMUM_OBSTACLE_POINTS else None
