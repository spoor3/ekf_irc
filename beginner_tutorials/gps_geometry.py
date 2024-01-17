#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' GPS Data: %s', data.data)

def arduino_com():
    arduino = serial.Serial('/dev/ttyACM0', 9600)
    pub = rospy.Publisher('setgoal_current', PoseStamped, queue_size=10)
    rospy.init_node('gps', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        if arduino.in_waiting > 0:
            gps_data = arduino.readline().decode().strip().split(' ')
            
            if len(gps_data) == 3:  # Assuming latitude, longitude, and altitude
                latitude, longitude = map(float, gps_data)
                
                # Create PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.pose.position.x = longitude
                pose_msg.pose.position.y = latitude


                rospy.loginfo("Publishing GPS data: {}".format(pose_msg))
                pub.publish(pose_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        arduino_com()
    except rospy.ROSInterruptException:
        pass
