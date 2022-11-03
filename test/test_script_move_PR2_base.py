import rospy
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion

# init ros node
rospy.init_node(u'test_pr2_base')
giskard_wrapper = GiskardWrapper()


root = u'map'

poses = [Point(0.1, 0, 0), Point(0.2, 0, 0), Point(0.3, 0, 0)]

#for pose in poses:

# Create a goal for the right hand
r_goal = PoseStamped()
r_goal.header.frame_id = 'base_link'
r_goal.header.stamp = rospy.get_rostime()
r_goal.pose.position = Point(-1.0, -1.0, 0)
r_goal.pose.orientation = Quaternion(1, 1, 1, 0)
giskard_wrapper.set_cart_goal(root_link=root, tip_link='base_link', goal_pose=r_goal)

giskard_wrapper.plan_and_execute()
