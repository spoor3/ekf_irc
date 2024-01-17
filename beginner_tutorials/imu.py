#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Vector3Stamped

# Define the publisher globally
pub = rospy.Publisher('imu_data', Vector3Stamped, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' IMU Data: %s', data.data)

    # Split the received data into roll, pitch, and yaw
    try:
        roll, pitch, yaw = map(float, data.data.split())
    except ValueError:
        rospy.logerr("Invalid IMU data format. Expected space-separated roll, pitch, and yaw.")
        return

    # Create a Vector3Stamped message
    vector_msg = Vector3Stamped()
    vector_msg.header.stamp = rospy.Time.now()
    vector_msg.vector.x = roll
    vector_msg.vector.y = pitch
    vector_msg.vector.z = yaw

    rospy.loginfo("Publishing IMU data as Vector3Stamped: {}".format(vector_msg))
    pub.publish(vector_msg)

def arduino_com():
    arduino = serial.Serial('/dev/ttyACM0', 9600)
    rospy.init_node('imu', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        if arduino.in_waiting > 0:
            imu_data_str = arduino.readline().decode().strip()
            rospy.loginfo(imu_data_str)
            callback(String(imu_data_str))  # Call the callback with a String message
            rate.sleep()

if __name__ == '__main__':
    try:
        arduino_com()
    except rospy.ROSInterruptException:
        pass
