#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = 0.5
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


# #!/usr/bin/env python

# import rospy
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib import SimpleActionClient
# from geometry_msgs.msg import Twist, PointStamped

# class setGoal:
#     def __init__(self):
#         self.command = Twist()
#         self.reached = False
#         self.stuck = False

#         self.vel_pub = rospy.Publisher('/fourbot/cmd_vel', Twist, queue_size=1)
#         self.rate = rospy.Rate(10)
#         self.start = rospy.Time.now().secs
#         rospy.loginfo("waiting for the server...")

#         # Tell the action client that we want to spin a thread by default
#         self.ac = SimpleActionClient('/move_base', MoveBaseAction)


#         # ps = PointStamped(point=point_wrt_kinect)
#         # pose_transformed = tf2_geometry_msgs.do_transform_point(ps, transform)

#         self.ac.wait_for_server()
#         self.goal = MoveBaseGoal()
#         self.goal.target_pose.header.frame_id = 'map'

#         while self.reached == False:
#             self.reset_goal()
#             wait = self.ac.wait_for_result()
#             if not wait:
#                 rospy.logerr("Action server not available!")
#                 rospy.signal_shutdown("Action server shutting down...")
#             else:
#                 rospy.loginfo("Goal execution complete!")
#                 self.reached = True
            
#         self.stop_rover()

#     def reset_goal(self):
#         # Send a goal to the robot to move 3 meter forward
#         self.goal.target_pose.header.frame_id = 'map'
#         self.goal.target_pose.header.stamp = rospy.Time.now()
#         self.goal.target_pose.pose.position.x = 6.0
#         self.goal.target_pose.pose.orientation.w = 1.0
#         # self.goal.target_pose.pose.orientation.w = self.goal.target_pose.pose.position.w + 1.0

#         rospy.loginfo("Resetting goal...")
#         self.ac.send_goal(self.goal)


#     def stop_rover(self):
#         self.command.linear.x = 0.0
#         self.command.linear.y = 0.0 
#         self.command.linear.z = 0.0 
#         self.command.angular.x = 0.0
#         self.command.angular.y = 0.0
#         self.command.angular.z = 0.0
#         rospy.signal_shutdown("Shutting down rover!")
        

# def main():
#     setGoal()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         rospy.loginfo("Keyboard interrupt, shutting down")


# if __name__ == "__main__":
#     try:
#         rospy.init_node('movebase_client')
#         main()
#         rospy.loginfo("Navigation function complete!")
#     except rospy.ROSInterruptException:
#         rospy.loginfo("ROS Exception, shutting down.")
