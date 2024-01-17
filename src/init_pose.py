# #!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry

# # Node initialization
# rospy.init_node('init_pose')
# pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)

# # Construct message
# init_msg = PoseWithCovarianceStamped()
# init_msg.header.frame_id = "robot_footprint"

# # Get initial pose from Gazebo
# odom_msg = rospy.wait_for_message('/odom', Odometry)
# init_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
# init_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
# init_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
# init_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
# init_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
# init_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

# # Delay
# rospy.sleep(1)

# # Publish message
# rospy.loginfo("setting initial pose")
# pub.publish(init_msg)
# rospy.loginfo("initial pose is set")

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Node initialization
rospy.init_node('init_pose')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

# Construct message
init_msg = PoseWithCovarianceStamped()
init_msg.header.frame_id = "map"  # Use "map" as the frame_id for the initial pose

# Get initial pose from Gazebo
try:
    odom_msg = rospy.wait_for_message('/odom', Odometry, timeout=5.0)  # Increase timeout if needed
except rospy.ROSException as e:
    rospy.logerr(f"Failed to receive /odom message: {e}")
    rospy.signal_shutdown("Failed to receive /odom message")
    exit(1)

init_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
init_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
init_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
init_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
init_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
init_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

# Delay (if needed)
rospy.sleep(1)

# Publish message
rospy.loginfo("Setting initial pose")
pub.publish(init_msg)
rospy.loginfo("Initial pose is set")
