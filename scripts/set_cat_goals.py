#!/usr/bin/env python
import rospy
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion
 
# init ros node
rospy.init_node(u'test')
giskard_wrapper = GiskardWrapper()
 
# Create a goal for the right hand
root = u'map'
r_goal = PoseStamped()
r_goal.header.frame_id = 'r_gripper_tool_frame'
r_goal.header.stamp = rospy.get_rostime()
r_goal.pose.position = Point(0.1,  0.1, -0.1)
r_goal.pose.orientation = Quaternion(0, 0, 0, 1)
giskard_wrapper.set_cart_goal(root, 'r_gripper_tool_frame', r_goal)

 
giskard_wrapper.plan_and_execute()
