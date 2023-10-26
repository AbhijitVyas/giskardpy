import rospy
import sys
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped
from giskardpy.python_interface import GiskardWrapper
from tf.transformations import quaternion_about_axis
import numpy as np
from numpy import pi
import json

# init ros node
rospy.init_node('test')
rospy.loginfo('Instantiating Giskard wrapper.')
giskard_wrapper = GiskardWrapper()

trajectoryPoints = [Point(-0.7666993737220764, 0.6055015921592712, 1.2316889762878418), 
                    Point(-0.7666993737220764,0.6055015921592712,1.2316889762878418), 
                    Point(-0.8500529527664185,0.6616386771202087,0.9167364239692688), 
                    Point(-0.8500529527664185,0.6616386771202087,0.9167364239692688), 
                    Point(-0.920591413974762,0.696370005607605,1.169901967048645),
                    Point(-0.956562876701355,0.6967198252677917,1.1744202375411987),
                    Point(-0.9444898962974548,0.70587158203125,1.1730669736862183),
                    Point(-0.7963405251502991,0.643972635269165,0.9390547871589661)]

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
    base_goal.pose.position = Point(3, 3, 0)
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
    #giskard_wrapper.add_urdf()
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

# Method prompts agent to move head of PR2 to left side.
# call this method from commandline with following command:
# python3 -c 'from interactive_commands import *; move_head_left()'
def move_head_left():
    rospy.loginfo('Move PR2 head left in -y direction.')
    base_goal = PoseStamped()
    # it is very important to use goal.header.frame_id as base_footprint 
    # so that the movements will be relative to the current base positions
    base_goal.header.frame_id = 'head_mount_kinect_rgb_link'
    base_goal.pose.position.y -= 0.5
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='head_mount_kinect_rgb_link', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    return giskard_wrapper.plan_and_execute()

# Method prompts agent to move head of PR2 to right side.
# call this method from commandline with following command:
# python3 -c 'from interactive_commands import *; move_head_right()'
def move_head_right():
    rospy.loginfo('Move PR2 head left in +y direction.')
    base_goal = PoseStamped()
    # it is very important to use goal.header.frame_id as base_footprint 
    # so that the movements will be relative to the current base positions
    base_goal.header.frame_id = 'head_mount_kinect_rgb_link'
    base_goal.pose.position.y += 0.5
    
    r_goal = PoseStamped()
    r_goal.header.frame_id = 'head_mount_kinect_rgb_link'
    r_goal.pose.orientation = Quaternion(*quaternion_about_axis(pi, [1, 0, 0]))
    
    # zero_pose.set_cart_goal(r_goal, zero_pose.r_tip)
    # zero_pose.plan_and_execute()
    
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='head_mount_kinect_rgb_link', goal_pose=base_goal)
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


def follow_human_right_hand():
# this method should make pr2 to follow right hand of the human from VR demonstrations.
# here first you calculate the difference between two points in human hand trajectory, such as diff = point2 - point1, in all 3 directions as well  as rotations
# then you can tranlate this difference into pr2's relative hand motions as follows:

    rospy.loginfo('Move right hand according to human hand trajectory.')
    # start imitation by following trajectory points.
    r_goal = PoseStamped()
    r_goal.header.frame_id = 'r_gripper_tool_frame'
    trajectoryPoses = readHandPosesFromJsonFile()
    for idx, tp in enumerate(trajectoryPoses):
        print('idx: ', idx)
        if idx + 1 < len(trajectoryPoses):
            curr_el = tp
            next_el = trajectoryPoses[idx+1]
            # print('\n', curr_el, "Next ->", next_el)
            current_position = curr_el[0]
            next_position = next_el[0]
            print("current pose: ", current_position, "\n")
            print("next_position: ", next_position,"\n")
            r_goal_pose_position = Point(next_position.x - current_position.x, next_position.y - current_position.y, next_position.z - current_position.z)
            print("r_goal_pose_position: ", r_goal_pose_position)
            # find difference between current point and next point so that PR2 only moves toward these differences.
            r_goal.pose.position = r_goal_pose_position 
            # no need to copy the orientation since the human hand will have different structure than PR2 gripper
            # r_goal.pose.orientation = next_rotation
            rospy.loginfo('Setting Cartesian goal.')
            giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='r_gripper_tool_frame', goal_pose=r_goal)
            # Turn off collision avoidance to make sure that the robot can recover from any state.
            giskard_wrapper.allow_all_collisions()
            giskard_wrapper.add_cmd()

    return giskard_wrapper.plan_and_execute()

    # base_goal = PoseStamped()
    # # it is very important to use goal.header.frame_id as base_footprint 
    # # so that the movements will be relative to the current base positions
    # base_goal.header.frame_id = 'r_gripper_tool_frame'
    # base_goal.pose.position.x = pose2.x - pose1.x
    # base_goal.pose.position.y = pose2.y - pose1.y
    # base_goal.pose.position.z = pose2.z - pose1.z
    # 
    # base_goal.pose.orientation = ori2
    # 
    # rospy.loginfo('Setting Cartesian goal.')
    # giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='r_gripper_tool_frame', goal_pose=base_goal)
    # # Turn off collision avoidance to make sure that the robot can recover from any state.
    # giskard_wrapper.allow_all_collisions()
    # return giskard_wrapper.plan_and_execute()


def readHandPosesFromJsonFile():
    # Opening JSON file
    f = open('/home/avyas/giskardpy_ws/src/giskardpy/scripts/handposes.json')
    
    # returns JSON object as 
    # a dictionary
    data = json.load(f)
    
    # Iterating through the json
    # list
    trajectoryPoses = []
    for i in data:
        translation = i['transform']['translation']
        rotation = i['transform']['rotation']
        trajectoryPoses.append([Point(translation['x'], translation['y'], translation['z']), 
                                Quaternion(rotation['x'], rotation['y'], rotation['z'], rotation['w'])])
    
    # Closing file
    f.close()
    
    return trajectoryPoses


def perform_pouring():
    # reset the world first
    reset()
    
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
    return giskard_wrapper.plan_and_execute()


def perform_pouring_in_apartment():
    cup_size = (0.1, 0.1, 0.2)
    cup_pose_point = Point(3.3, 3.7, 0.8)
    pr2_approach_pose_point = Point(2.3, 2, 0.8)
    pr2_pickup_pose_point = Point(2.3, 2, 1)

    # init ros node
    rospy.init_node('test')

    rospy.loginfo('Instantiating Giskard wrapper.')
    giskard_wrapper = GiskardWrapper()

    # it is very important to use goal.header.frame_id as r_gripper_tool_frame 
    # so that the movements will be relative to the current base positions
    base_goal = PoseStamped()
    base_goal.header.frame_id = 'r_gripper_tool_frame'
    base_goal.pose.position.z = +0.3
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='r_gripper_tool_frame', goal_pose=base_goal)

    base_goal.header.frame_id = 'l_gripper_tool_frame'
    base_goal.pose.position.z = +0.3
    rospy.loginfo('Setting Cartesian goal.')
    giskard_wrapper.set_cart_goal(root_link='base_footprint', tip_link='l_gripper_tool_frame', goal_pose=base_goal)
    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.plan_and_execute()
    
    # Move PR2 toward the box/table where human was standing or the object is needed to be grasped
    rospy.loginfo('Sending human position as a Cartesian goal for the base.')
    base_goal = PoseStamped()
    base_goal.header.frame_id = 'map'
    base_goal.pose.position.x = 4.0
    base_goal.pose.position.y = 3.5
    base_goal.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))

    # Setting the Cartesian goal.
    # Choosing map as root_link will allow Giskard to drive with the pr2.
    giskard_wrapper.set_cart_goal(root_link='map', tip_link='base_footprint', goal_pose=base_goal)

    # Turn off collision avoidance to make sure that the robot can recover from any state.
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.plan_and_execute()

    rospy.loginfo('Move right hand upward in +z direction.')
    base_goal = PoseStamped()
    