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

                     'torso_lift_joint': 0.9,
                     'head_pan_joint': 0,
                     'head_tilt_joint': 0}


table_box_size = (2, 1, 0.7)
cup_size = (0.1, 0.1, 0.2)
cup_pose_point = Point(2.3, 1.7, 0.8)
table_box_pose_point = Point(2, 2, 0.35)
pr2_approach_pose_point = Point(2.3, 2, 0.8)
pr2_pickup_pose_point = Point(2.3, 2, 1)

# init ros node
rospy.init_node('test')

rospy.loginfo('Instantiating Giskard wrapper.')
giskard_wrapper = GiskardWrapper()

# Remove everything but the robot.
giskard_wrapper.clear_world()

rospy.loginfo('Spawn a table box in the world.')
table_box_name = 'table_box'
table_box_pose = PoseStamped()
table_box_pose.header.frame_id = 'map'
table_box_pose.pose.position = table_box_pose_point
table_box_pose.pose.orientation.w = 0

giskard_wrapper.add_box(name=table_box_name,
                        size=table_box_size,
                        pose=table_box_pose,
                        parent_link='map')


# Span Cup as box in the world
rospy.loginfo('Spawn a cup box in the world.')
cup_box_name = 'cup_box'
cup_box_pose = PoseStamped()
cup_box_pose.header.frame_id = 'map'
cup_box_pose.pose.position = cup_pose_point
cup_box_pose.pose.orientation.w = 0

giskard_wrapper.add_box(name=cup_box_name,
                        size=cup_size,
                        pose=cup_box_pose,
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
# Setting the joint goal
giskard_wrapper.set_joint_goal(start_joint_state)

# Move PR2 toward the box/table where human was standing or the object is needed to be grasped
rospy.loginfo('Sending human position as a Cartesian goal for the base.')
base_goal = PoseStamped()
base_goal.header.frame_id = 'map'
base_goal.pose.position.x = 2
base_goal.pose.position.y = 0.8
base_goal.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 2, [0, 0, 1]))

# Setting the Cartesian goal.
# Choosing map as root_link will allow Giskard to drive with the pr2.
giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)

# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()


# Move PR2 right arm as approach position
rospy.loginfo('Move PR2 right arm as approach position.')
r_goal = PoseStamped()
r_goal.header.frame_id = 'map'
r_goal.pose.position = pr2_approach_pose_point
r_goal.pose.orientation = Quaternion(0, 0, 0, 1)
rospy.loginfo('Setting Cartesian goal.')
giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='r_gripper_tool_frame', goal_pose=r_goal)
# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()


# attach cup_box to pr2 right arm
rospy.loginfo('Attach cup box to pr2 right arm')
giskard_wrapper.update_parent_link_of_group(name=cup_box_name,
                                            parent_link='r_gripper_tool_frame')
cup_box_pose = PoseStamped()
cup_box_pose.header.frame_id = 'r_gripper_tool_frame'
cup_box_pose.pose.orientation.w = 1

rospy.loginfo('Set a Cartesian goal for the box')
box_goal = PoseStamped()
box_goal.header.frame_id = cup_box_name
box_goal.pose.orientation.w = 1
giskard_wrapper.set_cart_goal(goal_pose=box_goal,
                              tip_link=cup_box_name,
                              root_link='map')
# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()


# Move PR2 right arm as pickup position
rospy.loginfo('Move PR2 right arm as pickup position.')
r_goal = PoseStamped()
r_goal.header.frame_id = 'map'
r_goal.pose.position = pr2_pickup_pose_point
r_goal.pose.orientation = Quaternion(0, 0, 0, 1)
rospy.loginfo('Setting Cartesian goal.')
giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='r_gripper_tool_frame', goal_pose=r_goal)
# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()

# Move PR2 right arm as tilt position
rospy.loginfo('Move PR2 right arm as tilt position.')
r_goal = PoseStamped()
r_tip = 'r_gripper_tool_frame'
r_goal.header.frame_id = r_tip
r_goal.pose.orientation = Quaternion(*quaternion_about_axis(np.pi/1.5, [0, 1, 0]))
rospy.loginfo('Setting Cartesian goal.')
giskard_wrapper.set_cart_goal(root_link='base_footprint',tip_link=r_tip,goal_pose=r_goal)
# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()

rospy.loginfo('Letting pouring happen, sleep for 2 seconds.')
rospy.sleep(2)
# Move PR2 right arm as reverse tilt position
rospy.loginfo('Move PR2 right arm as reverse position.')
r_goal = PoseStamped()
r_tip = 'r_gripper_tool_frame'
r_goal.header.frame_id = r_tip
r_goal.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi/1.5, [0, 1, 0]))
rospy.loginfo('Setting Cartesian goal.')
giskard_wrapper.set_cart_goal(root_link='base_footprint',tip_link=r_tip,goal_pose=r_goal)
# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()

# Move PR2 right arm as putting down position
rospy.loginfo('Move PR2 right arm as putting down position.')
r_goal = PoseStamped()
r_goal.header.frame_id = 'map'
r_goal.pose.position = pr2_approach_pose_point
r_goal.pose.orientation = Quaternion(0, 0, 0, 1)
rospy.loginfo('Setting Cartesian goal.')
giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='r_gripper_tool_frame', goal_pose=r_goal)
# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()

# Detach the cup box from pr2 arm
rospy.loginfo('Delete only the box in order to detach it from pr2 hand.')
giskard_wrapper.remove_group(name=cup_box_name)
rospy.loginfo('Spawn a cup box in the world.')
cup_box_pose = PoseStamped()
cup_box_pose.header.frame_id = 'map'
cup_box_pose.pose.position = cup_pose_point
cup_box_pose.pose.orientation.w = 0

giskard_wrapper.add_box(name=cup_box_name,
                        size=cup_size,
                        pose=cup_box_pose,
                        parent_link='map')

# Move PR2 right arm as release backward position
rospy.loginfo('Move PR2 right arm as release backward position.')
# Setting the Cartesian goal.
# Choosing map as root_link will allow Giskard to drive with the pr2.
giskard_wrapper.set_joint_goal(start_joint_state)
giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)

# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()
