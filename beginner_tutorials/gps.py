#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' GPS Data: %s', data.data)
def arduino_com():
    arduino=serial.Serial('/dev/ttyUSB0', 9600)
    pub = rospy.Publisher('gps_data', String, queue_size=10)
    rospy.init_node('gps', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if arduino.in_waiting>0:
            hello_str=arduino.readline().decode().strip()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()

if __name__ == '__main__':
    try:
        arduino_com()
    except rospy.ROSInterruptException:
        pass
