#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf
import time


class NavGoalMonitor:
    def __init__(self):
        self.nav_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_goal_callback)
        self.odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)
        self.nav_goal = None
        self.robot_pose = None
        self.goal_reached = False
        self.tf_listener = tf.TransformListener()
        self.last_position = None
        self.last_position_time = None
        self.status_timer = rospy.Timer(rospy.Duration(1), self.status_callback)

    def nav_goal_callback(self, msg):
        self.nav_goal = msg.pose
        self.goal_reached = False
        self.last_position = None
        self.last_position_time = None
        print("New navigation goal received. (x: {:.2f}, y: {:.2f})".format(msg.pose.position.x, msg.pose.position.y))

    def odom_callback(self, msg):
        try:
            self.tf_listener.waitForTransform("map", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            transformed_pose = self.tf_listener.transformPose("map", PoseStamped(header=msg.header, pose=msg.pose.pose))
            self.robot_pose = transformed_pose.pose
            self.check_stopped()
            self.check_goal_reached()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(str(e))

    def check_stopped(self):
        if self.robot_pose and self.goal_reached == False:
            current_position = self.robot_pose.position
            if self.last_position:
                distance_moved = calculate_distance(current_position, self.last_position)
                if distance_moved < 0.01:
                    if (time.time() - self.last_position_time) > 5:
                        rospy.loginfo("Navigation failed")
                        self.goal_reached = True
                else:
                    self.last_position_time = time.time()
            else:
                self.last_position_time = time.time()
            self.last_position = current_position

    def check_goal_reached(self):
        if self.nav_goal and self.robot_pose and not self.goal_reached:
            distance = calculate_distance(self.robot_pose.position, self.nav_goal.position)
            angle_diff = calculate_angle_difference(self.robot_pose.orientation, self.nav_goal.orientation)

            distance_threshold = 0.5
            angle_threshold = 0.1

            if distance < distance_threshold and angle_diff < angle_threshold:
                rospy.loginfo("Goal reached")
                self.goal_reached = True

    def status_callback(self, event):
        if self.nav_goal and self.robot_pose and self.goal_reached == False:
            distance = calculate_distance(self.robot_pose.position, self.nav_goal.position)
            print("Robot position: x: {:.2f}, y: {:.2f}".format(
                self.robot_pose.position.x, self.robot_pose.position.y))
            print("Goal position: x: {:.2f}, y: {:.2f}".format(
                self.nav_goal.position.x, self.nav_goal.position.y))
            print("Distance to goal: {:.2f} meters".format(distance))

def calculate_distance(position1, position2):
    dx = position1.x - position2.x
    dy = position1.y - position2.y
    return (dx**2 + dy**2)**0.5


def calculate_angle_difference(orientation1, orientation2):
    _, _, yaw1 = euler_from_quaternion([orientation1.x, orientation1.y, orientation1.z, orientation1.w])
    _, _, yaw2 = euler_from_quaternion([orientation2.x, orientation2.y, orientation2.z, orientation2.w])
    return abs(yaw1 - yaw2)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_goal_monitor')
        monitor = NavGoalMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
