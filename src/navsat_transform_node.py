#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from robot_localization import NavSatTransform
from rospy.impl.init import init_node

class NavSatTransformNode(rospy.Node):
    def __init__(self):
        super(NavSatTransformNode, self).__init__('navsat_transform_node')
        self.navsat_transform_node = NavSatTransform(self)

    def spin(self) -> None:
        rospy.spin()

if __name__ == '__main__':
    try:
        init_node('navsat_transform_node')
        node = NavSatTransformNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
