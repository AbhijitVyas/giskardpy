import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped
from giskardpy.python_interface import GiskardWrapper
from tf.transformations import quaternion_about_axis
import numpy as np

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

# init ros node
rospy.init_node('test')

rospy.loginfo('Instantiating Giskard wrapper.')
giskard_wrapper = GiskardWrapper()

# Remove everything but the robot.
giskard_wrapper.clear_world()

rospy.loginfo('Spawn a box in the world.')
box_name = 'muh'
box_pose = PoseStamped()
box_pose.header.frame_id = 'map'
box_pose.pose.position.x = 2
box_pose.pose.position.y = 2
box_pose.pose.position.z = 0.5
box_pose.pose.orientation.w = 0

giskard_wrapper.add_box(name=box_name,
                        size=(2, 1, 0.7),
                        pose=box_pose,
                        parent_link='map')

base_goal = PoseStamped()
base_goal.header.frame_id = 'map'
base_goal.pose.position = Point(0, 0, 0)
base_goal.pose.orientation = Quaternion(0, 0, 0, 1)
# Setting the Cartesian goal.
# Choosing map as root_link will allow Giskard to drive with the pr2.
rospy.loginfo('Reset the pr2 to origin.')
giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)
# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()

rospy.loginfo('Sending human position as a Cartesian goal for the base to reset the pr2.')
# Setting the joint goal
giskard_wrapper.set_joint_goal(start_joint_state)

base_goal = PoseStamped()
base_goal.header.frame_id = 'map'
base_goal.pose.position.x = 2
base_goal.pose.position.y = 1
base_goal.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 2, [0, 0, 1]))

# Setting the Cartesian goal.
# Choosing map as root_link will allow Giskard to drive with the pr2.
giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)

# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()

# Move PR2 toward the box

# Move PR2 right arm as approach position

# Move PR2 right arm as pickup position

# Move PR2 right arm as tilt position

# Move PR2 right arm as reverse tilt position

# Move PR2 right arm as puttingdown position

# Move PR2 right arm as release backward position




# rospy.loginfo('Set a Cartesian goal for the box')
# box_goal = PoseStamped()
# box_goal.header.frame_id = box_name
# box_goal.pose.position.x = 4
# box_goal.pose.position.y = 4
# box_goal.pose.position.z = 2
# box_goal.pose.orientation.w = 0
# giskard_wrapper.set_cart_goal(goal_pose=box_goal,
#                               tip_link=box_name,
#                               root_link='map')
# giskard_wrapper.plan_and_execute()
# 
