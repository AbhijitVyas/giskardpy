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

# create goal joint state dictionary
start_joint_state = {'r_elbow_flex_joint': -1.29610152504,
                     'r_forearm_roll_joint': -0.0301682323805,
                     'r_shoulder_lift_joint': 1.20324921318,
                     'r_shoulder_pan_joint': -0.73456435706,
                     'r_upper_arm_roll_joint': -0.70790051778,
                     'r_wrist_flex_joint': -0.10001,
                     'r_wrist_roll_joint': 0.258268529825,

                     'l_elbow_flex_joint': -1.29610152504,
                     'l_forearm_roll_joint': 0.0301682323805,
                     'l_shoulder_lift_joint': 1.20324921318,
                     'l_shoulder_pan_joint': 0.73456435706,
                     'l_upper_arm_roll_joint': 0.70790051778,
                     'l_wrist_flex_joint': -0.1001,
                     'l_wrist_roll_joint': -0.258268529825,

                     'torso_lift_joint': 0.2,
                     'head_pan_joint': 0,
                     'head_tilt_joint': 0}


def reset():
    # Remove everything but the robot.
    giskard_wrapper.clear_world()
    base_goal = PoseStamped()
    base_goal.header.frame_id = 'map'
    base_goal.pose.position = Point(0, 0, 0)
    base_goal.pose.orientation = Quaternion(0, 0, 0, 1)
    # Setting the Cartesian goal.
    # Choosing map as root_link will allow Giskard to drive with the pr2.
    rospy.loginfo('Reset the pr2 to origin.')
    giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)
    # Setting the joint goal
    giskard_wrapper.set_joint_goal(start_joint_state)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    return giskard_wrapper.plan_and_execute()


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
    return giskard_wrapper.plan_and_execute()


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
    return giskard_wrapper.plan_and_execute()


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
    return giskard_wrapper.plan_and_execute()


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
    return giskard_wrapper.plan_and_execute()


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
    return giskard_wrapper.plan_and_execute()


# Method prompts agent to raise right hand.
# call this method from commandline with following command:
# python3 -c 'from interactive_commands import *; raise_right_hand()'
def raise_right_hand():
    rospy.loginfo('Move right hand upward in +z direction.')
    base_goal = PoseStamped()
    # it is very important to use goal.header.frame_id as r_gripper_tool_frame 
    # so that the movements will be relative to the current base positions
    base_goal.header.frame_id = 'r_gripper_tool_frame'
    base_goal.pose.position.z = +0.3
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='r_gripper_tool_frame', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    return giskard_wrapper.plan_and_execute()


# Method prompts agent to raise left hand.
# call this method from commandline with following command:
# python3 -c 'from interactive_commands import *; raise_left_hand()'
def raise_left_hand():
    rospy.loginfo('Move left hand upward in +z direction.')
    base_goal = PoseStamped()
    # it is very important to use goal.header.frame_id as base_footprint 
    # so that the movements will be relative to the current base positions
    base_goal.header.frame_id = 'l_gripper_tool_frame'
    base_goal.pose.position.z = +0.3
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='l_gripper_tool_frame', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    return giskard_wrapper.plan_and_execute()

# TODO: method: grasp_object_with_right_hand(pose):
# this method should make pr2 to grasp the object with right hand at give position


def follow_human_right_hand(pose1, pose2, ori2):
# this method should make pr2 to follow right hand of the human from VR demonstrations.
# here first you calculate the difference between two points in human hand trajectory, such as diff = point2 - point1, in all 3 directions as well  as rotations
# then you can tranlate this difference into pr2's relative hand motions as follows:

    rospy.loginfo('Move right hand according to human hand.')
    base_goal = PoseStamped()
    # it is very important to use goal.header.frame_id as base_footprint 
    # so that the movements will be relative to the current base positions
    base_goal.header.frame_id = 'r_gripper_tool_frame'
    base_goal.pose.position.x = pose2.x - pose1.x
    base_goal.pose.position.y = pose2.y - pose1.y
    base_goal.pose.position.z = pose2.z - pose1.z

    base_goal.pose.orientation = ori2

    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='r_gripper_tool_frame', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    return giskard_wrapper.plan_and_execute()



