import rospy
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from knowrobTfListener import  returnTrajectoryPointVector
import numpy as np
from test_integration_pr2 import TestConstraints
from utils_for_tests import PR2
class Test:
    def __init__(self):
        TestConstraints.test_CartesianPosition(self, zero_pose=PR2)

# points = returnTrajectoryPointVector()
#
# # init ros node
# rospy.init_node(u'test_pr2_arm')
# giskard_wrapper = GiskardWrapper()
# # Create a goal for moving the base
# rootMap = u'map'
# print("All the points from test script", points)
# for point in points:
#     print("for each individual point ", point)
#     rootBase = u'base_link'
#     r_goal = PoseStamped()
#     r_goal.header.frame_id = 'r_gripper_tool_frame'
#     r_goal.header.stamp = rospy.get_rostime()
#     r_goal.pose.position = Point(point[0], point[1], point[2])
#     r_goal.pose.orientation = Quaternion(0, 0, 0, 1)
#     giskard_wrapper.set_cart_goal(root_link=rootBase, tip_link='r_gripper_tool_frame', goal_pose=r_goal)
#     giskard_wrapper.add_cmd()
#
# giskard_wrapper.plan_and_execute()

# loop through all the points

#for point in points:
#    print("I am executing point: ", point)
    # Create a goal for the right hand
#    rootBase = u'base_link'
#    r_goal = PoseStamped()
#    r_goal.header.frame_id = 'r_gripper_tool_frame'
#    r_goal.header.stamp = rospy.get_rostime()
#    r_goal.pose.position = Point(point[0], point[1], point[2])
#    r_goal.pose.orientation = Quaternion(0, 0, 0, 1)
#    giskard_wrapper.set_cart_goal(root_link=rootBase, tip_link='r_gripper_tool_frame', goal_pose=r_goal)



#
# # Create a goal for the left hand
# l_goal = PoseStamped()
# l_goal.header.frame_id = 'l_gripper_tool_frame'
# l_goal.header.stamp = rospy.get_rostime()
# l_goal.pose.position = Point(0.05, 0, 0)
# l_goal.pose.orientation = Quaternion(0, 0, 0, 1)
# giskard_wrapper.set_cart_goal(root_link=root, tip_link='l_gripper_tool_frame', goal_pose=l_goal)
#
# giskard_wrapper.add_cmd()
#
# # Create a goal for the right hand
# root = u'base_link'
# r_goal = PoseStamped()
# r_goal.header.frame_id = 'r_gripper_tool_frame'
# r_goal.header.stamp = rospy.get_rostime()
# r_goal.pose.position = Point(0.2, 0, 0)
# r_goal.pose.orientation = Quaternion(0, 0, 0, 1)
# giskard_wrapper.set_cart_goal(root_link=root, tip_link='r_gripper_tool_frame', goal_pose=r_goal)
#
# # Create a goal for the left hand
# l_goal = PoseStamped()
# l_goal.header.frame_id = 'l_gripper_tool_frame'
# l_goal.header.stamp = rospy.get_rostime()
# l_goal.pose.position = Point(0.01, 0, 0)
# l_goal.pose.orientation = Quaternion(0, 0, 0, 1)
# giskard_wrapper.set_cart_goal(root_link=root, tip_link='l_gripper_tool_frame', goal_pose=l_goal)
#
# giskard_wrapper.add_cmd()
#
# # Create a goal for the right hand
# root = u'base_link'
# r_goal = PoseStamped()
# r_goal.header.frame_id = 'r_gripper_tool_frame'
# r_goal.header.stamp = rospy.get_rostime()
# r_goal.pose.position = Point(0.3, 0, 0)
# r_goal.pose.orientation = Quaternion(0, 0, 0, 1)
# giskard_wrapper.set_cart_goal(root_link=root, tip_link='r_gripper_tool_frame', goal_pose=r_goal)
#
# # Create a goal for the left hand
# l_goal = PoseStamped()
# l_goal.header.frame_id = 'l_gripper_tool_frame'
# l_goal.header.stamp = rospy.get_rostime()
# l_goal.pose.position = Point(0.02, 0, 0)
# l_goal.pose.orientation = Quaternion(0, 0, 0, 1)
# giskard_wrapper.set_cart_goal(root_link=root, tip_link='l_gripper_tool_frame', goal_pose=l_goal)

