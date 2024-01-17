#!/usr/bin/env python3

from turtle import setpos
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from math import radians, cos, sin

class GPSGoalSetter:
    def _init_(self):
        rospy.Subscriber('gps_coordinates', PointStamped, self.gps_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.navclient.wait_for_server()

    def gps_callback(self, data):
        # Convert latitude and longitude to Cartesian coordinates
        x, y = self.lat_lon_to_cartesian(data.point.y, data.point.x)

        # Log the Cartesian coordinates for debugging
        rospy.loginfo("GPS Coordinates (lat={}, lon={}): Converted to Cartesian (x={}, y={})".format(
            data.point.y, data.point.x, x, y))

        # Publish Twist message for robot movement
        twist_msg = Twist()
        twist_msg.linear.x = 1000 * x  # Adjust the sensitivity as needed
        twist_msg.linear.y = 1000 * y  # Adjust the sensitivity as needed
        self.vel_pub.publish(twist_msg)

        # Set the navigation goal based on the GPS coordinates
        self.set_navigation_goal(x, y)

    def lat_lon_to_cartesian(self, latitude, longitude):
        # Convert latitude and longitude to Cartesian coordinates (in meters)
        R = 6371000  # Earth radius in meters

        # Convert degrees to radians
        lat_rad = radians(latitude)
        lon_rad = radians(longitude)

        # Apply haversine formula to calculate Cartesian coordinates
        x = R * cos(lat_rad) * cos(lon_rad)
        y = R * cos(lat_rad) * sin(lon_rad)

        return x, y

    def set_navigation_goal(self, x, y):
        # Set the navigation goal based on the GPS coordinates
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.662
        goal.target_pose.pose.orientation.w = 0.750

        rospy.loginfo("Setting navigation goal to x={}, y={}".format(x, y))
        self.navclient.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def active_cb(self, extra):
        rospy.loginfo("Goal pose being processed")

    def feedback_cb(self, feedback):
        rospy.loginfo("Current location: " + str(feedback))

    def done_cb(self, status, result):
        if status == 3:
            rospy.loginfo("Goal reached")
        elif status == 2 or status == 8:
            rospy.loginfo("Goal cancelled")
        elif status == 4:
            rospy.loginfo("Goal aborted")

def main():
    rospy.init_node('gps_goal_setter')
    gps_goal_setter = GPSGoalSetter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt, shutting down")

if __name__ == "__main__":
    main()