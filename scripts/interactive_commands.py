import rospy
import sys
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped
from giskardpy.python_interface import GiskardWrapper
from tf.transformations import quaternion_about_axis
import numpy as np

# init ros node
rospy.init_node('test')
rospy.loginfo('Instantiating Giskard wrapper.')
giskard_wrapper = GiskardWrapper()


def clear_world():
    # Remove everything but the robot.
    giskard_wrapper.clear_world()


# Method prompts agent to move to given pose.
# call this method from commandline with following command:
# python3 -c 'from interactive_commands import *; move(Point(1.0, 2.0,0))'
def move(pose):
    rospy.loginfo('Move to %s given position.', pose)
    base_goal = PoseStamped()
    # since goal.header.frame_id is map, all the pose positions will be relative to map
    base_goal.header.frame_id = 'map'
    base_goal.pose.position = pose
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.plan_and_execute()


# Method prompts agent to move to given pose.
# call this method from commandline with following command:
# python3 -c 'from interactive_commands import *; move_forward()'
def move_forward():
    rospy.loginfo('Move forward in +x direction.')
    base_goal = PoseStamped()
    # it is very important to use goal.header.frame_id as base_footprint 
    # so that the movements will be relative to the current base positions
    base_goal.header.frame_id = 'base_footprint'
    base_goal.pose.position.x += 1
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.plan_and_execute()


# Method prompts agent to move to given pose.
# call this method from commandline with following command:
# python3 -c 'from interactive_commands import *; move_backward()'
def move_backward():
    rospy.loginfo('Move backward in -x direction.')
    base_goal = PoseStamped()
    # it is very important to use goal.header.frame_id as base_footprint 
    # so that the movements will be relative to the current base positions
    base_goal.header.frame_id = 'base_footprint'
    base_goal.pose.position.x = -1
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.plan_and_execute()


# Method prompts agent to move to given pose.
# call this method from commandline with following command:
# python3 -c 'from interactive_commands import *; move_left()'
def move_left():
    rospy.loginfo('Move left in +y direction.')
    base_goal = PoseStamped()
    # it is very important to use goal.header.frame_id as base_footprint 
    # so that the movements will be relative to the current base positions 
    base_goal.header.frame_id = 'base_footprint'
    base_goal.pose.position.y = 1
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.plan_and_execute()


# Method prompts agent to move to given pose.
# call this method from commandline with following command:
# python3 -c 'from interactive_commands import *; move_right()'
def move_right():
    rospy.loginfo('Move right in -y direction.')
    base_goal = PoseStamped()
    # it is very important to use goal.header.frame_id as base_footprint 
    # so that the movements will be relative to the current base positions
    base_goal.header.frame_id = 'base_footprint'
    base_goal.pose.position.y = -1
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.plan_and_execute()
