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

pocky_pose = {'r_elbow_flex_joint': -1.29610152504,
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
              'head_tilt_joint': 0,
              }

pick_up_pose = {
    'head_pan_joint': -2.46056758502e-16,
    'head_tilt_joint': -1.97371778181e-16,
    'l_elbow_flex_joint': -0.962150355946,
    'l_forearm_roll_joint': 1.44894622393,
    'l_shoulder_lift_joint': -0.273579583084,
    'l_shoulder_pan_joint': 0.0695426768038,
    'l_upper_arm_roll_joint': 1.3591238067,
    'l_wrist_flex_joint': -1.9004529902,
    'l_wrist_roll_joint': 2.23732576003,
    'r_elbow_flex_joint': -2.1207193579,
    'r_forearm_roll_joint': 1.76628402882,
    'r_shoulder_lift_joint': -0.256729037039,
    'r_shoulder_pan_joint': -1.71258744959,
    'r_upper_arm_roll_joint': -1.46335011257,
    'r_wrist_flex_joint': -0.100010762609,
    'r_wrist_roll_joint': 0.0509923457388,
    'torso_lift_joint': 0.261791330751,
}

better_pose = {'r_shoulder_pan_joint': -1.7125,
               'r_shoulder_lift_joint': -0.25672,
               'r_upper_arm_roll_joint': -1.46335,
               'r_elbow_flex_joint': -2.12,
               'r_forearm_roll_joint': 1.76632,
               'r_wrist_flex_joint': -0.10001,
               'r_wrist_roll_joint': 0.05106,
               'l_shoulder_pan_joint': 1.9652,
               'l_shoulder_lift_joint': - 0.26499,
               'l_upper_arm_roll_joint': 1.3837,
               'l_elbow_flex_joint': -2.12,
               'l_forearm_roll_joint': 16.99,
               'l_wrist_flex_joint': - 0.10001,
               'l_wrist_roll_joint': 0,
               'torso_lift_joint': 0.2,
               'l_gripper_l_finger_joint': 0.55,
               'r_gripper_l_finger_joint': 0.55,
               'head_pan_joint': 0,
               'head_tilt_joint': 0,
}


# init ros node
rospy.init_node('test')

rospy.loginfo('Instantiating Giskard wrapper.')
giskard_wrapper = GiskardWrapper()
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
# Turn off collision avoidance to make sure that the robot can recover from any state.
giskard_wrapper.allow_all_collisions()
giskard_wrapper.plan_and_execute()

rospy.loginfo('Spawn a box')
cup_box_name = 'cup_box'
cup_box_pose = PoseStamped()
cup_box_pose.header.frame_id = 'r_gripper_tool_frame'
cup_box_pose.pose.orientation.w = 1
giskard_wrapper.add_box(name=cup_box_name,
                        size=(0.1, 0.1, 0.2),
                        pose=cup_box_pose,
                        parent_link='map')

rospy.loginfo('Attach it to the robot')
giskard_wrapper.update_parent_link_of_group(name=cup_box_name,
                                            parent_link='r_gripper_tool_frame')

trajectoryPoints = []
pr2_pose_point_1 = Point(0.5, -0.2, 0.8)
pr2_pose_point_2 = Point(0.5, 0.0, 1)
pr2_pose_point_3 = Point(0.5, 0.2, 0.8)
pr2_orientation_1 = Quaternion(*quaternion_about_axis(0, [1, 0, 0]))
pr2_orientation_2 = Quaternion(*quaternion_about_axis(-np.pi / 2, [1, 0, 0]))
pr2_orientation_3 = Quaternion(*quaternion_about_axis(-np.pi / 1.5, [1, 0, 0]))

trajectoryPoints.append([pr2_pose_point_1, pr2_orientation_1])
trajectoryPoints.append([pr2_pose_point_2, pr2_orientation_2])
trajectoryPoints.append([pr2_pose_point_3, pr2_orientation_3])

# start imitation by following trajectory points.
for tp in trajectoryPoints:
    rospy.loginfo('Move to %s position.', tp)
    r_goal = PoseStamped()
    r_goal.header.frame_id = 'map'
    r_goal.pose.position = tp[0]
    r_goal.pose.orientation = tp[1]
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='r_gripper_tool_frame', goal_pose=r_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.add_cmd()

giskard_wrapper.plan_and_execute()


