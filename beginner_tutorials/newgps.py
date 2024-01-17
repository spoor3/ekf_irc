#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

# Define the publisher globally
pub = rospy.Publisher('gps_coordinates', PointStamped, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' GPS Data: %s', data.data)

    # Split the received data into latitude and longitude
    try:
        latitude, longitude = map(float, data.data.split())
    except ValueError:
        rospy.logerr("Invalid GPS data format. Expected space-separated latitude and longitude.")
        return

    # Create a PointStamped message
    point_msg = PointStamped()
    point_msg.header.stamp = rospy.Time.now()
    point_msg.point.x = longitude
    point_msg.point.y = latitude

    rospy.loginfo("Publishing GPS data as PointStamped: {}".format(point_msg))
    pub.publish(point_msg)

def arduino_com():
    arduino = serial.Serial('/dev/ttyACM0', 9600)
    rospy.init_node('gps', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        if arduino.in_waiting > 0:
            hello_str = arduino.readline().decode().strip()
            rospy.loginfo(hello_str)
            callback(String(hello_str))  # Call the callback with a String message
            rate.sleep()

if __name__ == '__main__':
    try:
        arduino_com()
    except rospy.ROSInterruptException:
        pass



