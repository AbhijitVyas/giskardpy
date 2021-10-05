from time import time
import keyword
from collections import defaultdict
from copy import deepcopy
from multiprocessing import Queue
from threading import Thread
from multiprocessing.pool import ThreadPool
import hypothesis.strategies as st
import numpy as np
import rospy
from actionlib_msgs.msg import GoalID
from angles import shortest_angular_distance
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from giskard_msgs.msg import MoveActionResult, CollisionEntry, MoveActionGoal, MoveResult, MoveGoal
from giskard_msgs.srv import UpdateWorldResponse
from hypothesis import assume
from hypothesis.strategies import composite
from iai_naive_kinematics_sim.srv import SetJointState, SetJointStateRequest
from iai_wsg_50_msgs.msg import PositionCmd
from numpy import pi
from py_trees import Blackboard
from rospy import Timer
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from tf.transformations import rotation_from_matrix, quaternion_matrix
from visualization_msgs.msg import Marker

from giskardpy import identifier, RobotName, RobotPrefix
from giskardpy.data_types import KeyDefaultDict, JointStates, PrefixName
from giskardpy.utils import logging
from giskardpy.utils.config_loader import ros_load_robot_config
from giskardpy.garden import grow_tree
from giskardpy.identifier import robot, world
from giskardpy.model.pybullet_world import PyBulletWorld
from giskardpy.python_interface import GiskardWrapper
from giskardpy.model.robot import Robot
from giskardpy.utils.tfwrapper import transform_pose, lookup_pose, np_to_pose
from giskardpy.utils.utils import msg_to_list, position_dict_to_joint_states, to_joint_state_position_dict

BIG_NUMBER = 1e100
SMALL_NUMBER = 1e-100

vector = lambda x: st.lists(float_no_nan_no_inf(), min_size=x, max_size=x)

update_world_error_codes = {value: name for name, value in vars(UpdateWorldResponse).items() if
                            isinstance(value, int) and name[0].isupper()}


def update_world_error_code(code):
    return update_world_error_codes[code]


move_result_error_codes = {value: name for name, value in vars(MoveResult).items() if
                           isinstance(value, int) and name[0].isupper()}


def move_result_error_code(code):
    return move_result_error_codes[code]


def robot_urdfs():
    return st.sampled_from([u'urdfs/pr2.urdfs', u'urdfs/boxy.urdfs', u'urdfs/iai_donbot.urdfs'])
    # return st.sampled_from([u'pr2.urdfs'])


def angle_positive():
    return st.floats(0, 2 * np.pi)


def angle():
    return st.floats(-np.pi, np.pi)


def keys_values(max_length=10, value_type=st.floats(allow_nan=False)):
    return lists_of_same_length([variable_name(), value_type], max_length=max_length, unique=True)


def compare_axis_angle(actual_angle, actual_axis, expected_angle, expected_axis, decimal=3):
    try:
        np.testing.assert_array_almost_equal(actual_axis, expected_axis, decimal=decimal)
        np.testing.assert_almost_equal(shortest_angular_distance(actual_angle, expected_angle), 0, decimal=decimal)
    except AssertionError:
        try:
            np.testing.assert_array_almost_equal(actual_axis, -expected_axis, decimal=decimal)
            np.testing.assert_almost_equal(shortest_angular_distance(actual_angle, abs(expected_angle - 2 * pi)), 0,
                                           decimal=decimal)
        except AssertionError:
            np.testing.assert_almost_equal(shortest_angular_distance(actual_angle, 0), 0, decimal=decimal)
            np.testing.assert_almost_equal(shortest_angular_distance(0, expected_angle), 0, decimal=decimal)
            assert not np.any(np.isnan(actual_axis))
            assert not np.any(np.isnan(expected_axis))


def compare_poses(pose1, pose2, decimal=2):
    """
    :type pose1: Pose
    :type pose2: Pose
    """
    compare_points(pose1.position, pose2.position, decimal)
    compare_orientations(pose1.orientation, pose2.orientation, decimal)

def compare_points(actual_point, desired_point, decimal=2):
    """
    :type pose1: Point
    :type pose2: Point
    """
    np.testing.assert_almost_equal(actual_point.x, desired_point.x, decimal=decimal)
    np.testing.assert_almost_equal(actual_point.y, desired_point.y, decimal=decimal)
    np.testing.assert_almost_equal(actual_point.z, desired_point.z, decimal=decimal)


def compare_orientations(orientation1, orientation2, decimal=2):
    """
    :type orientation1: Quaternion
    :type orientation2: Quaternion
    """
    if isinstance(orientation1, Quaternion):
        q1 = np.array([orientation1.x,
                       orientation1.y,
                       orientation1.z,
                       orientation1.w])
    else:
        q1 = orientation1
    if isinstance(orientation2, Quaternion):
        q2 = np.array([orientation2.x,
                       orientation2.y,
                       orientation2.z,
                       orientation2.w])
    else:
        q2 = orientation2
    try:
        np.testing.assert_almost_equal(q1[0], q2[0], decimal=decimal)
        np.testing.assert_almost_equal(q1[1], q2[1], decimal=decimal)
        np.testing.assert_almost_equal(q1[2], q2[2], decimal=decimal)
        np.testing.assert_almost_equal(q1[3], q2[3], decimal=decimal)
    except:
        np.testing.assert_almost_equal(q1[0], -q2[0], decimal=decimal)
        np.testing.assert_almost_equal(q1[1], -q2[1], decimal=decimal)
        np.testing.assert_almost_equal(q1[2], -q2[2], decimal=decimal)
        np.testing.assert_almost_equal(q1[3], -q2[3], decimal=decimal)


@composite
def variable_name(draw):
    variable = draw(st.text(u'qwertyuiopasdfghjklzxcvbnm', min_size=1))
    assume(variable not in keyword.kwlist)
    return variable


@composite
def lists_of_same_length(draw, data_types=(), min_length=1, max_length=10, unique=False):
    length = draw(st.integers(min_value=min_length, max_value=max_length))
    lists = []
    for elements in data_types:
        lists.append(draw(st.lists(elements, min_size=length, max_size=length, unique=unique)))
    return lists


@composite
def rnd_joint_state(draw, joint_limits):
    return {jn: draw(st.floats(ll, ul, allow_nan=False, allow_infinity=False)) for jn, (ll, ul) in joint_limits.items()}


@composite
def rnd_joint_state2(draw, joint_limits):
    muh = draw(joint_limits)
    muh = {jn: ((ll if ll is not None else pi * 2), (ul if ul is not None else pi * 2))
           for (jn, (ll, ul)) in muh.items()}
    return {jn: draw(st.floats(ll, ul, allow_nan=False, allow_infinity=False)) for jn, (ll, ul) in muh.items()}


@composite
def pr2_joint_state(draw):
    pr2 = Robot.from_urdf_file(pr2_urdf())
    return draw(rnd_joint_state(*pr2.get_joint_position_limits()))


def pr2_urdf():
    with open(u'urdfs/pr2_with_base.urdf', u'r') as f:
        urdf_string = f.read()
    return urdf_string


def pr2_without_base_urdf():
    with open(u'urdfs/pr2.urdf', u'r') as f:
        urdf_string = f.read()
    return urdf_string


def base_bot_urdf():
    with open(u'urdfs/2d_base_bot.urdf', u'r') as f:
        urdf_string = f.read()
    return urdf_string


def donbot_urdf():
    with open(u'urdfs/iai_donbot.urdf', u'r') as f:
        urdf_string = f.read()
    return urdf_string


def boxy_urdf():
    with open(u'urdfs/boxy.urdf', u'r') as f:
        urdf_string = f.read()
    return urdf_string

def hsr_urdf():
    with open(u'urdfs/hsr.urdf', u'r') as f:
        urdf_string = f.read()
    return urdf_string


def float_no_nan_no_inf(outer_limit=None, min_dist_to_zero=None):
    if outer_limit is not None:
        return st.floats(allow_nan=False, allow_infinity=False, max_value=outer_limit, min_value=-outer_limit)
    else:
        return st.floats(allow_nan=False, allow_infinity=False)
    # f = st.floats(allow_nan=False, allow_infinity=False, max_value=outer_limit, min_value=-outer_limit)
    # # f = st.floats(allow_nan=False, allow_infinity=False)
    # if min_dist_to_zero is not None:
    #     f = f.filter(lambda x: (outer_limit > abs(x) and abs(x) > min_dist_to_zero) or x == 0)
    # else:
    #     f = f.filter(lambda x: abs(x) < outer_limit)
    # return f


@composite
def sq_matrix(draw):
    i = draw(st.integers(min_value=1, max_value=10))
    i_sq = i ** 2
    l = draw(st.lists(float_no_nan_no_inf(outer_limit=1000), min_size=i_sq, max_size=i_sq))
    return np.array(l).reshape((i, i))


def unit_vector(length, elements=None):
    if elements is None:
        elements = float_no_nan_no_inf(min_dist_to_zero=1e-10)
    vector = st.lists(elements,
                      min_size=length,
                      max_size=length).filter(lambda x: np.linalg.norm(x) > SMALL_NUMBER and
                                                        np.linalg.norm(x) < BIG_NUMBER)

    def normalize(v):
        v = [round(x, 4) for x in v]
        l = np.linalg.norm(v)
        if l == 0:
            return np.array([0] * (length - 1) + [1])
        return np.array([x / l for x in v])

    return st.builds(normalize, vector)


def quaternion(elements=None):
    return unit_vector(4, elements)


def pykdl_frame_to_numpy(pykdl_frame):
    return np.array([[pykdl_frame.M[0, 0], pykdl_frame.M[0, 1], pykdl_frame.M[0, 2], pykdl_frame.p[0]],
                     [pykdl_frame.M[1, 0], pykdl_frame.M[1, 1], pykdl_frame.M[1, 2], pykdl_frame.p[1]],
                     [pykdl_frame.M[2, 0], pykdl_frame.M[2, 1], pykdl_frame.M[2, 2], pykdl_frame.p[2]],
                     [0, 0, 0, 1]])


class GoalChecker(object):
    def __init__(self, god_map):
        """
        :type god_map: giskardpy.god_map.GodMap
        """
        self.god_map = god_map
        self.world = self.god_map.unsafe_get_data(identifier.world)
        self.robot = self.god_map.unsafe_get_data(identifier.robot)


class JointGoalChecker(GoalChecker):
    def __init__(self, god_map, goal_state, decimal=2):
        super(JointGoalChecker, self).__init__(god_map)
        self.goal_state = goal_state
        self.decimal = decimal

    def get_current_joint_state(self):
        """
        :rtype: JointState
        """
        return rospy.wait_for_message('joint_states', JointState)

    def __call__(self):
        current_joint_state = JointStates.from_msg(self.get_current_joint_state())
        self.compare_joint_state(current_joint_state, self.goal_state, decimal=self.decimal)

    def compare_joint_state(self, current_js, goal_js, decimal=2):
        """
        :type current_js: dict
        :type goal_js: dict
        :type decimal: int
        """
        for joint_name in goal_js:
            goal = goal_js[joint_name]
            current = current_js[PrefixName(joint_name, None)].position
            if self.world.is_joint_continuous(PrefixName(joint_name, RobotPrefix)):
                np.testing.assert_almost_equal(shortest_angular_distance(goal, current), 0, decimal=decimal,
                                               err_msg=u'{}: actual: {} desired: {}'.format(joint_name, current,
                                                                                            goal))
            else:
                np.testing.assert_almost_equal(current, goal, decimal,
                                               err_msg=u'{}: actual: {} desired: {}'.format(joint_name, current,
                                                                                            goal))


class CartGoalChecker(GoalChecker):
    def __init__(self, god_map, tip_link, root_link, expected):
        super(CartGoalChecker, self).__init__(god_map)
        self.expected = expected
        self.tip_link = tip_link
        self.root_link = root_link
        self.expected = transform_pose(self.root_link, self.expected)

    def __call__(self):
        expected = self.expected
        current_pose = lookup_pose(self.root_link, self.tip_link)
        np.testing.assert_array_almost_equal(msg_to_list(expected.pose.position),
                                             msg_to_list(current_pose.pose.position), decimal=2)

        try:
            np.testing.assert_array_almost_equal(msg_to_list(expected.pose.orientation),
                                                 msg_to_list(current_pose.pose.orientation), decimal=2)
        except AssertionError:
            np.testing.assert_array_almost_equal(msg_to_list(expected.pose.orientation),
                                                 -np.array(msg_to_list(current_pose.pose.orientation)), decimal=2)

class GiskardTestWrapper(GiskardWrapper):
    def __init__(self, config_file):
        # self.pool = ThreadPool(processes=1)
        self.total_time_spend_giskarding = 0
        self.total_time_spend_moving = 0

        if not ros_load_robot_config(config_file, test=True):
            rospy.logerr('Could not set robot config as ROS parameter.')
        rospy.set_param('~tree/PlotDebugTrajectory/enabled', True)
        rospy.set_param('~tree/MaxTrajectoryLength/enabled', True)

        self.sub_result = rospy.Subscriber('~command/result', MoveActionResult, self.cb, queue_size=100)
        self.start_motion_sub = rospy.Subscriber('/whole_body_controller/follow_joint_trajectory/goal',
                                                FollowJointTrajectoryActionGoal, self.start_motion_cb,
                                                 queue_size=100)
        self.stop_motion_sub = rospy.Subscriber('/whole_body_controller/follow_joint_trajectory/result',
                                               FollowJointTrajectoryActionResult, self.stop_motion_cb,
                                                queue_size=100)

        self.tree = grow_tree()
        self.tick_rate = self.get_god_map().unsafe_get_data(identifier.tree_tick_rate)
        self.heart = Timer(rospy.Duration(self.tick_rate), self.live)
        # sleeper = rospy.Rate(self.tick_rate)
        # self.tick_once()
        # rospy.sleep(1)
        super(GiskardTestWrapper, self).__init__(node_name=u'tests')
        self.results = Queue(100)
        self.default_root = self.get_robot().root_link_name.short_name
        self.map = u'map'
        self.simple_base_pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.set_base = rospy.ServiceProxy('/base_simulator/set_joint_states', SetJointState)
        self.goal_checks = defaultdict(list)


        def create_publisher(topic):
            p = rospy.Publisher(topic, JointState, queue_size=10)
            rospy.sleep(.2)
            return p

        self.joint_state_publisher = KeyDefaultDict(create_publisher)
        # rospy.sleep(1)

    @property
    def bullet(self):
        """
        :rtype: giskardpy.model.pybullet_syncer.PyBulletSyncer
        """
        return self.get_god_map().unsafe_get_data(identifier.bullet)

    @property
    def robot_self_collision_matrix(self):
        """
        :rtype: set
        """
        return self.bullet.collision_matrices[RobotName]

    def start_motion_cb(self, msg):
        self.time = time()

    def stop_motion_cb(self, msg):
        self.total_time_spend_moving += time() - self.time

    def wait_for_synced(self):
        sleeper = rospy.Rate(self.tick_rate)
        self.tick_once()
        # while self.tree.tip().name != u'has goal':
        #     self.loop_once()
        #     sleeper.sleep()
        # self.loop_once()
        # while self.tree.tip().name != u'has goal':
        #     self.loop_once()
        #     sleeper.sleep()

    def get_robot(self):
        """
        :rtype: giskardpy.model.world.SubWorldTree
        """
        return self.get_world().groups['robot']

    def get_god_map(self):
        """
        :rtype: giskardpy.god_map.GodMap
        """
        return Blackboard().god_map

    def cb(self, msg):
        """
        :type msg: MoveActionResult
        """
        self.results.put(msg.result)

    def tick_once(self):
        pass
        # self.tree.tick()

    def tick(self, times=1):
        pass
        # for _ in range(times):
        #     self.tick_once()

    def live(self, timer_thing):
        self.tree.tick()

    def get_controlled_joint_names(self):
        """
        :rtype: dict
        """
        return self.get_robot().controlled_joints

    def get_controllable_links(self):
        return self.get_robot().get_controlled_links()

    def tear_down(self):
        rospy.sleep(1)
        logging.loginfo('wtf')
        self.heart.shutdown()
        logging.loginfo(u'total time spend giskarding: {}'.format(self.total_time_spend_giskarding - self.total_time_spend_moving))
        logging.loginfo(u'total time spend moving: {}'.format(self.total_time_spend_moving))
        logging.loginfo(u'stopping tree')

    def set_object_joint_state(self, object_name, joint_state):
        super(GiskardTestWrapper, self).set_object_joint_state(object_name, joint_state)
        self.tick_once()
        self.tick_once()
        rospy.sleep(0.5)
        self.wait_for_synced()
        current_js = self.get_world().groups[object_name].state
        joint_names_without_prefix = set(j.short_name for j in current_js)
        assert set(joint_state.keys()).difference(joint_names_without_prefix) == set()
        for joint_name, state in current_js.items():
            if joint_name.short_name in joint_state:
                np.testing.assert_almost_equal(state.position, joint_state[joint_name.short_name], 2)

    def set_kitchen_js(self, joint_state, object_name=u'kitchen'):
        self.set_object_joint_state(object_name, joint_state)

    #
    # JOINT GOAL STUFF #################################################################################################
    #

    def set_joint_goal(self, goal, weight=None, decimal=2, expected_error_codes=(MoveResult.SUCCESS,), check=True):
        """
        :type goal: dict
        """
        super(GiskardTestWrapper, self).set_joint_goal(goal, weight=weight)
        if check:
            self.add_goal_check(JointGoalChecker(self.get_god_map(), goal, decimal))

    #
    # CART GOAL STUFF ##################################################################################################
    #
    def teleport_base(self, goal_pose):
        goal_pose = transform_pose(self.default_root, goal_pose)
        js = {u'odom_x_joint': goal_pose.pose.position.x,
              u'odom_y_joint': goal_pose.pose.position.y,
              u'odom_z_joint': rotation_from_matrix(quaternion_matrix([goal_pose.pose.orientation.x,
                                                                       goal_pose.pose.orientation.y,
                                                                       goal_pose.pose.orientation.z,
                                                                       goal_pose.pose.orientation.w]))[0]}
        goal = SetJointStateRequest()
        goal.state = position_dict_to_joint_states(js)
        self.set_base.call(goal)
        self.tick_once()
        rospy.sleep(0.5)
        self.tick_once()

    def keep_position(self, tip, root=None):
        if root is None:
            root = self.default_root
        goal = PoseStamped()
        goal.header.frame_id = tip
        goal.pose.orientation.w = 1
        self.set_cart_goal(goal, tip, root)

    def keep_orientation(self, tip, root=None):
        goal = PoseStamped()
        goal.header.frame_id = tip
        goal.pose.orientation.w = 1
        self.set_rotation_goal(goal, tip, root)

    def set_rotation_goal(self, goal_pose, tip_link, root_link=None, weight=None, max_velocity=None, **kwargs):
        if not root_link:
            root_link = self.default_root
        super(GiskardTestWrapper, self).set_rotation_goal(goal_pose, tip_link, root_link, max_velocity=max_velocity,
                                                          weight=weight, **kwargs)

    def set_translation_goal(self, goal_pose, tip_link, root_link=None, weight=None, max_velocity=None, **kwargs):
        if not root_link:
            root_link = self.default_root
        super(GiskardTestWrapper, self).set_translation_goal(goal_pose, tip_link, root_link, max_velocity=max_velocity,
                                                             weight=weight, **kwargs)


    def set_straight_translation_goal(self, goal_pose, tip_link, root_link=None, weight=None, max_velocity=None, **kwargs):
        if not root_link:
            root_link = self.default_root
        super(GiskardTestWrapper, self).set_straight_translation_goal(goal_pose, tip_link, root_link,
                                                                      max_velocity=max_velocity, weight=weight,
                                                                      **kwargs)

    def set_cart_goal(self, goal_pose, tip_link, root_link=None, weight=None, linear_velocity=None, angular_velocity=None,
                      check=True):
        if not root_link:
            root_link = self.default_root

        if weight is not None:
            super(GiskardTestWrapper, self).set_cart_goal(goal_pose,
                                                          tip_link,
                                                          root_link,
                                                          weight=weight,
                                                          max_linear_velocity=linear_velocity,
                                                          max_angular_velocity=angular_velocity)
        else:
            super(GiskardTestWrapper, self).set_cart_goal(goal_pose,
                                                          tip_link,
                                                          root_link,
                                                          max_linear_velocity=linear_velocity,
                                                          max_angular_velocity=angular_velocity)

        if check:
            # map_T_goal = transform_pose(u'map', goal_pose)
            self.add_goal_check(CartGoalChecker(self.get_god_map(), tip_link, root_link, goal_pose))

    def add_goal_check(self, goal_checker):
        self.goal_checks[self.number_of_cmds-1].append(goal_checker)

    def set_straight_cart_goal(self, goal_pose, tip_link, root_link=None, weight=None, linear_velocity=None, angular_velocity=None):
        if not root_link:
            root_link = self.default_root
        if weight is not None:
            super(GiskardTestWrapper, self).set_straight_cart_goal(goal_pose, tip_link, root_link, weight=weight, max_linear_velocity=linear_velocity,
                                                                   max_angular_velocity=angular_velocity)
        else:
            super(GiskardTestWrapper, self).set_straight_cart_goal(goal_pose, tip_link, root_link, max_linear_velocity=linear_velocity,
                                                                   max_angular_velocity=angular_velocity)


    # def set_and_check_cart_goal(self, goal_pose, tip_link, root_link=None, weight=None, linear_velocity=None, angular_velocity=None,
    #                             expected_error_codes=None):
    #     self.set_cart_goal(goal_pose, tip_link, root_link, weight, linear_velocity=linear_velocity, angular_velocity=angular_velocity)
    #     goal_pose_in_map = transform_pose(u'map', goal_pose)
    #     self.tick_once()
    #     self.send_goal(expected_error_codes)
    #     self.tick_once()
    #     def goal_checker():
    #         if expected_error_codes is None:
    #             self.check_cart_goal(tip_link, goal_pose_in_map)


    def set_and_check_straight_cart_goal(self, goal_pose, tip_link, root_link=None, weight=None, linear_velocity=None, angular_velocity=None,
                                expected_error_codes=None):
        goal_pose_in_map = transform_pose(u'map', goal_pose)
        self.set_straight_cart_goal(goal_pose, tip_link, root_link, weight, linear_velocity=linear_velocity, angular_velocity=angular_velocity)
        self.tick_once()
        self.send_goal(expected_error_codes)
        self.tick_once()
        if expected_error_codes is None:
            self.check_cart_goal(tip_link, goal_pose_in_map)



    #
    # GENERAL GOAL STUFF ###############################################################################################
    #

    # def interrupt(self):
    #     self.cancel_goal.publish(GoalID())

    def check_reachability(self, expected_error_codes=None):
        self.send_and_check_goal(expected_error_codes=expected_error_codes,
                                 goal_type=MoveGoal.PLAN_AND_CHECK_REACHABILITY)

    def get_as(self):
        return Blackboard().get(u'~command')

    # def send_goal(self, goal=None, goal_type=MoveGoal.PLAN_AND_EXECUTE, wait=True):
    #     """
    #     :rtype: MoveResult
    #     """
    #     # if goal is None:
    #     #     goal = MoveActionGoal()
    #     #     goal.goal = self._get_goal()
    #     #     goal.goal.type = goal_type
    #     i = 0
    #     # self.tick_once()
    #     t = time()
    #     sleeper = rospy.Rate(self.tick_rate)
    #     # self.tick_once()
    #     # self.pool.apply_async(self.get_as()._as.action_server.internal_goal_callback, (goal,))
    #     # self.get_as()._as.action_server.internal_goal_callback(goal)
    #     super(GiskardTestWrapper, self).plan_and_execute(goal)
    #     # while self.results.empty():
    #     #     self.tick_once()
    #         # sleeper.sleep()
    #     # t = time() - t
    #     # self.total_time_spend_giskarding += t
    #     # self.tick_once()
    #     result = self.results.get()
    #     return result

    def plan_and_execute(self, expected_error_codes=None, stop_after=None):
        return self.send_goal(expected_error_codes=expected_error_codes, stop_after=stop_after)

    def send_goal(self, expected_error_codes=None, goal_type=MoveGoal.PLAN_AND_EXECUTE, goal=None, stop_after=None):
        try:
            time_spend_giskarding = time()
            if stop_after is None:
                r = super(GiskardTestWrapper, self).send_goal(goal_type, wait=True)
            else:
                super(GiskardTestWrapper, self).send_goal(goal_type, wait=False)
                rospy.sleep(stop_after)
                self.interrupt()
                rospy.sleep(1)
                r = self.get_result(rospy.Duration(3))
            self.total_time_spend_giskarding += time() - time_spend_giskarding
            for cmd_id in range(len(r.error_codes)):
                error_code = r.error_codes[cmd_id]
                error_message = r.error_messages[cmd_id]
                if expected_error_codes is None:
                    expected_error_code = MoveResult.SUCCESS
                else:
                    expected_error_code = expected_error_codes[cmd_id]
                assert error_code == expected_error_code, \
                    u'in goal {}; got: {}, expected: {} | error_massage: {}'.format(cmd_id, move_result_error_code(error_code),
                                                                                    move_result_error_code(
                                                                                        expected_error_code),
                                                                                    error_message)
                if error_code == MoveResult.SUCCESS:
                    try:
                        for goal_checker in self.goal_checks[cmd_id]:
                            goal_checker()
                    except:
                        logging.logerr('Goal #{} did\'t pass test.'.format(cmd_id))
                        raise
            self.are_joint_limits_violated()
        finally:
            self.goal_checks = defaultdict(list)
        return r.trajectory

    def get_result_trajectory_position(self):
        trajectory = self.get_god_map().unsafe_get_data(identifier.trajectory)
        trajectory2 = {}
        for joint_name in trajectory.get_exact(0).keys():
            trajectory2[joint_name] = np.array([p[joint_name].position for t, p in trajectory.items()])
        return trajectory2

    def get_result_trajectory_velocity(self):
        trajectory = self.get_god_map().get_data(identifier.trajectory)
        trajectory2 = {}
        for joint_name in trajectory.get_exact(0).keys():
            trajectory2[joint_name] = np.array([p[joint_name].velocity for t, p in trajectory.items()])
        return trajectory2

    def are_joint_limits_violated(self):
        trajectory_vel = self.get_result_trajectory_velocity()
        trajectory_pos = self.get_result_trajectory_position()
        controlled_joints = self.get_god_map().get_data(identifier.controlled_joints)
        for joint in controlled_joints:
            if not self.get_robot().is_joint_continuous(joint):
                joint_limits = self.get_robot().get_joint_position_limits(joint)
                error_msg = u'{} has violated joint position limit'.format(joint)
                np.testing.assert_array_less(trajectory_pos[joint], joint_limits[1], error_msg)
                np.testing.assert_array_less(-trajectory_pos[joint], -joint_limits[0], error_msg)
            vel_limit = self.get_world().joint_limit_expr(joint, 1)[1]
            vel_limit = self.get_god_map().evaluate_expr(vel_limit) * 1.001
            vel = trajectory_vel[joint]
            error_msg = u'{} has violated joint velocity limit {} > {}'.format(joint, vel, vel_limit)
            assert np.all(np.less_equal(vel, vel_limit)), error_msg
            assert np.all(np.greater_equal(vel, -vel_limit)), error_msg

    #
    # BULLET WORLD #####################################################################################################
    #
    def get_world(self):
        """
        :rtype: giskardpy.model.world.WorldTree
        """
        return self.get_god_map().get_data(world)

    def clear_world(self):
        return_val = super(GiskardTestWrapper, self).clear_world()
        assert return_val.error_codes == UpdateWorldResponse.SUCCESS
        assert len(self.get_world().groups) == 1
        assert len(self.get_object_names().object_names) == 1

    def remove_object(self, name, expected_response=UpdateWorldResponse.SUCCESS):
        old_link_names = self.get_world().groups[name].link_names
        old_joint_names = self.get_world().groups[name].joint_names
        r = super(GiskardTestWrapper, self).remove_object(name)
        self.tick_once()
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        assert not name in self.get_world().groups
        for old_link_name in old_link_names:
            assert old_link_name not in self.get_world().link_names
        for old_joint_name in old_joint_names:
            assert old_joint_name not in self.get_world().joint_names
        assert not name in self.get_object_names().object_names

    def detach_object(self, name, expected_response=UpdateWorldResponse.SUCCESS):
        if expected_response == UpdateWorldResponse.SUCCESS:
            p = self.get_robot().compute_fk_pose(self.get_robot().root_link_name, name)
            p = transform_pose(u'map', p)
            assert name in self.get_attached_objects().object_names, \
                'there is no attached object named {}'.format(name)
        r = super(GiskardTestWrapper, self).detach_object(name)
        self.tick_once()
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        if expected_response == UpdateWorldResponse.SUCCESS:
            assert name in self.get_world().groups

    def add_box(self, name=u'box', size=(1, 1, 1), pose=None, expected_response=UpdateWorldResponse.SUCCESS):
        r = super(GiskardTestWrapper, self).add_box(name, size, pose=pose)
        self.tick_once()
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        if expected_response == UpdateWorldResponse.SUCCESS:
            p = transform_pose(u'map', pose)
            o_p = self.get_world().groups[name].base_pose
            compare_poses(p.pose, o_p)
            assert name in self.get_object_names().object_names
            compare_poses(o_p, self.get_object_info(name).pose.pose)
        else:
            assert name not in self.get_world().groups
            assert name not in self.get_object_names().object_names

    def add_sphere(self, name=u'sphere', size=1., pose=None, expected_response=UpdateWorldResponse.SUCCESS):
        r = super(GiskardTestWrapper, self).add_sphere(name=name, size=size, pose=pose)
        self.tick_once()
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        if expected_response == UpdateWorldResponse.SUCCESS:
            p = transform_pose(u'map', pose)
            o_p = self.get_world().groups[name].base_pose
            compare_poses(p.pose, o_p)
            assert name in self.get_object_names().object_names
            compare_poses(o_p, self.get_object_info(name).pose.pose)
        else:
            assert name not in self.get_world().groups
            assert name not in self.get_object_names().object_names

    def add_cylinder(self, name=u'cylinder', size=[1, 1], pose=None, expected_response=UpdateWorldResponse.SUCCESS):
        r = super(GiskardTestWrapper, self).add_cylinder(name=name, height=size[0], radius=size[1], pose=pose)
        self.tick_once()
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        if expected_response == UpdateWorldResponse.SUCCESS:
            p = transform_pose(u'map', pose)
            o_p = self.get_world().groups[name].base_pose
            compare_poses(p.pose, o_p)
            assert name in self.get_object_names().object_names
            compare_poses(o_p, self.get_object_info(name).pose.pose)
        else:
            assert name not in self.get_world().groups
            assert name not in self.get_object_names().object_names

    def add_mesh(self, name=u'meshy', path=u'', pose=None, expected_response=UpdateWorldResponse.SUCCESS):
        r = super(GiskardTestWrapper, self).add_mesh(name=name, mesh=path, pose=pose)
        self.tick_once()
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        if expected_response == UpdateWorldResponse.SUCCESS:
            p = transform_pose(u'map', pose)
            o_p = self.get_world().groups[name].base_pose
            compare_poses(p.pose, o_p)
            assert name in self.get_object_names().object_names
            compare_poses(o_p, self.get_object_info(name).pose.pose)
        else:
            assert name not in self.get_world().groups
            assert name not in self.get_object_names().object_names

    def add_urdf(self, name, urdf, pose, js_topic=u'', set_js_topic=None, expected_response=UpdateWorldResponse.SUCCESS):
        r = super(GiskardTestWrapper, self).add_urdf(name, urdf, pose, js_topic, set_js_topic=set_js_topic)
        self.tick_once()
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        if expected_response == UpdateWorldResponse.SUCCESS:
            p = transform_pose(u'map', pose)
            o_p = self.get_world().groups[name].base_pose
            compare_poses(p.pose, o_p)
            assert name in self.get_object_names().object_names
            compare_poses(o_p, self.get_object_info(name).pose.pose)
        else:
            assert name not in self.get_world().groups
            assert name not in self.get_object_names().object_names

    def avoid_all_collisions(self, distance=0.5):
        super(GiskardTestWrapper, self).avoid_all_collisions(distance)
        self.tick_once()

    def attach_box(self, name=u'box', size=None, frame_id=None, position=None, orientation=None, pose=None,
                   expected_response=UpdateWorldResponse.SUCCESS):

        scm = self.robot_self_collision_matrix
        if pose is None:
            expected_pose = PoseStamped()
            expected_pose.header.frame_id = frame_id
            expected_pose.pose.position = Point(*position)
            if orientation:
                expected_pose.pose.orientation = Quaternion(*orientation)
            else:
                expected_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        else:
            expected_pose = deepcopy(pose)
        r = super(GiskardTestWrapper, self).attach_box(name, size, frame_id, position, orientation, pose)
        self.tick_once()
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        if expected_response == UpdateWorldResponse.SUCCESS:
            self.wait_for_synced()
            assert name in [n.short_name for n in self.get_robot().link_names]
            assert scm.difference(self.robot_self_collision_matrix) == set()
            assert len(scm) < len(self.robot_self_collision_matrix)
            compare_poses(expected_pose.pose, lookup_pose(frame_id, name).pose)
        self.tick_once()

    def attach_cylinder(self, name=u'cylinder', height=1, radius=1, frame_id=None, position=None, orientation=None,
                        pose=None, expected_response=UpdateWorldResponse.SUCCESS):
        scm = self.robot_self_collision_matrix
        if pose is None:
            expected_pose = PoseStamped()
            expected_pose.header.frame_id = frame_id
            expected_pose.pose.position = Point(*position)
            if orientation:
                expected_pose.pose.orientation = Quaternion(*orientation)
            else:
                expected_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        else:
            expected_pose = deepcopy(pose)
        r = super(GiskardTestWrapper, self).attach_cylinder(name, height, radius, frame_id, position, orientation)
        self.tick_once()
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        if expected_response == UpdateWorldResponse.SUCCESS:
            self.wait_for_synced()
            assert name in [n.short_name for n in self.get_robot().link_names]
            assert scm.difference(self.robot_self_collision_matrix) == set()
            assert len(scm) < len(self.robot_self_collision_matrix)
            compare_poses(expected_pose.pose, lookup_pose(frame_id, name).pose)
        self.tick_once()

    def attach_object(self, name=u'box', frame_id=None, expected_response=UpdateWorldResponse.SUCCESS):
        scm = self.robot_self_collision_matrix
        r = super(GiskardTestWrapper, self).attach_object(name, frame_id)
        assert r.error_codes == expected_response, \
            u'got: {}, expected: {}'.format(update_world_error_code(r.error_codes),
                                            update_world_error_code(expected_response))
        self.wait_for_synced()
        # assert name in self.get_controllable_links()
        assert name in self.get_attached_objects().object_names
        assert scm.difference(self.robot_self_collision_matrix) == set()
        assert len(scm) < len(self.robot_self_collision_matrix)
        self.tick_once()

    def get_external_collisions(self, link, distance_threshold):
        """
        :param distance_threshold:
        :rtype: list
        """
        collision_goals = [CollisionEntry(type=CollisionEntry.AVOID_ALL_COLLISIONS, min_dist=distance_threshold)]
        collision_matrix = self.bullet.collision_goals_to_collision_matrix(collision_goals,
                                                                                defaultdict(lambda: 0.3))
        collisions = self.bullet.check_collisions(collision_matrix)
        controlled_parent_joint = self.get_robot().get_controlled_parent_joint_of_link(link)
        controlled_parent_link = self.get_robot().joints[controlled_parent_joint].child_link_name
        collision_list = collisions.get_external_collisions(controlled_parent_link)
        for key, self_collisions in collisions.self_collisions.items():
            if controlled_parent_link in key:
                collision_list.update(self_collisions)
        return collision_list

    def check_cpi_geq(self, links, distance_threshold):
        for link in links:
            collisions = self.get_external_collisions(link, distance_threshold)
            assert collisions[0].contact_distance >= distance_threshold, \
                u'distance for {}: {} >= {}'.format(link,
                                                    collisions[0].contact_distance,
                                                    distance_threshold)

    def check_cpi_leq(self, links, distance_threshold):
        for link in links:
            collisions = self.get_external_collisions(link, distance_threshold)
            assert collisions[0].contact_distance <= distance_threshold, \
                u'distance for {}: {} <= {}'.format(link,
                                                    collisions[0].contact_distance,
                                                    distance_threshold)

    def move_base(self, goal_pose):
        """
        :type goal_pose: PoseStamped
        """
        self.simple_base_pose_pub.publish(goal_pose)
        rospy.sleep(.07)
        self.wait_for_synced()
        current_pose = self.get_robot().get_base_pose()
        goal_pose = transform_pose(u'map', goal_pose)
        compare_poses(goal_pose.pose, current_pose.pose, decimal=1)

    def reset_base(self):
        p = PoseStamped()
        p.header.frame_id = self.map
        p.pose.orientation.w = 1
        self.teleport_base(p)


class PR2(GiskardTestWrapper):
    def __init__(self):
        self.r_tip = u'r_gripper_tool_frame'
        self.l_tip = u'l_gripper_tool_frame'
        self.r_gripper = rospy.ServiceProxy(u'r_gripper_simulator/set_joint_states', SetJointState)
        self.l_gripper = rospy.ServiceProxy(u'l_gripper_simulator/set_joint_states', SetJointState)
        super(PR2, self).__init__(u'pr2.yaml')

    def move_base(self, goal_pose):
        goal_pose = transform_pose(self.default_root, goal_pose)
        js = {u'odom_x_joint': goal_pose.pose.position.x,
              u'odom_y_joint': goal_pose.pose.position.y,
              u'odom_z_joint': rotation_from_matrix(quaternion_matrix([goal_pose.pose.orientation.x,
                                                                       goal_pose.pose.orientation.y,
                                                                       goal_pose.pose.orientation.z,
                                                                       goal_pose.pose.orientation.w]))[0]}
        self.send_and_check_joint_goal(js)

    def get_l_gripper_links(self):
        return [u'l_gripper_l_finger_tip_link', u'l_gripper_r_finger_tip_link', u'l_gripper_l_finger_link',
                u'l_gripper_r_finger_link', u'l_gripper_r_finger_link', u'l_gripper_palm_link']

    def get_r_gripper_links(self):
        return [u'r_gripper_l_finger_tip_link', u'r_gripper_r_finger_tip_link', u'r_gripper_l_finger_link',
                u'r_gripper_r_finger_link', u'r_gripper_r_finger_link', u'r_gripper_palm_link']

    def get_r_upper_arm(self):
        return [u'r_shoulder_lift_link', u'r_upper_arm_roll_link', u'r_upper_arm_link']

    def get_r_forearm_links(self):
        return [u'r_wrist_flex_link', u'r_wrist_roll_link', u'r_forearm_roll_link', u'r_forearm_link',
                u'r_forearm_link']

    def get_allow_l_gripper(self, body_b=u'box'):
        links = self.get_l_gripper_links()
        return [CollisionEntry(CollisionEntry.ALLOW_COLLISION, 0, [link], body_b, []) for link in links]

    def get_l_gripper_collision_entries(self, body_b=u'box', distance=0, action=CollisionEntry.ALLOW_COLLISION):
        links = self.get_l_gripper_links()
        return [CollisionEntry(action, distance, [link], body_b, []) for link in links]

    def open_r_gripper(self):
        sjs = SetJointStateRequest()
        sjs.state.name = [u'r_gripper_l_finger_joint', u'r_gripper_r_finger_joint', u'r_gripper_l_finger_tip_joint',
                          u'r_gripper_r_finger_tip_joint']
        sjs.state.position = [0.54, 0.54, 0.54, 0.54]
        sjs.state.velocity = [0, 0, 0, 0]
        sjs.state.effort = [0, 0, 0, 0]
        self.r_gripper.call(sjs)

    def close_r_gripper(self):
        sjs = SetJointStateRequest()
        sjs.state.name = [u'r_gripper_l_finger_joint', u'r_gripper_r_finger_joint', u'r_gripper_l_finger_tip_joint',
                          u'r_gripper_r_finger_tip_joint']
        sjs.state.position = [0, 0, 0, 0]
        sjs.state.velocity = [0, 0, 0, 0]
        sjs.state.effort = [0, 0, 0, 0]
        self.r_gripper.call(sjs)

    def open_l_gripper(self):
        sjs = SetJointStateRequest()
        sjs.state.name = [u'l_gripper_l_finger_joint', u'l_gripper_r_finger_joint', u'l_gripper_l_finger_tip_joint',
                          u'l_gripper_r_finger_tip_joint']
        sjs.state.position = [0.54, 0.54, 0.54, 0.54]
        sjs.state.velocity = [0, 0, 0, 0]
        sjs.state.effort = [0, 0, 0, 0]
        self.l_gripper.call(sjs)

    def close_l_gripper(self):
        sjs = SetJointStateRequest()
        sjs.state.name = [u'l_gripper_l_finger_joint', u'l_gripper_r_finger_joint', u'l_gripper_l_finger_tip_joint',
                          u'l_gripper_r_finger_tip_joint']
        sjs.state.position = [0, 0, 0, 0]
        sjs.state.velocity = [0, 0, 0, 0]
        sjs.state.effort = [0, 0, 0, 0]
        self.l_gripper.call(sjs)

    def move_pr2_base(self, goal_pose):
        """
        :type goal_pose: PoseStamped
        """
        self.simple_base_pose_pub.publish(goal_pose)

    def reset_pr2_base(self):
        p = PoseStamped()
        p.header.frame_id = self.map
        p.pose.orientation.w = 1
        self.move_pr2_base(p)


class Donbot(GiskardTestWrapper):
    def __init__(self):
        self.camera_tip = u'camera_link'
        self.gripper_tip = u'gripper_tool_frame'
        self.gripper_pub = rospy.Publisher(u'/wsg_50_driver/goal_position', PositionCmd, queue_size=10)
        super(Donbot, self).__init__(u'donbot.yaml')

    def move_base(self, goal_pose):
        goal_pose = transform_pose(self.default_root, goal_pose)
        js = {u'odom_x_joint': goal_pose.pose.position.x,
              u'odom_y_joint': goal_pose.pose.position.y,
              u'odom_z_joint': rotation_from_matrix(quaternion_matrix([goal_pose.pose.orientation.x,
                                                                       goal_pose.pose.orientation.y,
                                                                       goal_pose.pose.orientation.z,
                                                                       goal_pose.pose.orientation.w]))[0]}
        self.allow_all_collisions()
        self.send_and_check_joint_goal(js)

    def open_gripper(self):
        self.set_gripper(0.109)

    def close_gripper(self):
        self.set_gripper(0)

    def set_gripper(self, width, gripper_joint=u'gripper_joint'):
        """
        :param width: goal width in m
        :type width: float
        """
        width = max(0.0065, min(0.109, width))
        goal = PositionCmd()
        goal.pos = width * 1000
        self.gripper_pub.publish(goal)
        rospy.sleep(0.5)
        js = self.get_current_joint_state()
        index = js.name.index(gripper_joint)
        np.testing.assert_almost_equal(js.position[index], width, decimal=3)


class KMR_IIWA(GiskardTestWrapper):
    def __init__(self):
        self.camera_tip = u'camera_link'
        self.gripper_tip = u'gripper_tool_frame'
        super(KMR_IIWA, self).__init__(u'kmr_iiwa.yaml')

    def move_base(self, goal_pose):
        goal_pose = transform_pose(self.default_root, goal_pose)
        js = {u'odom_x_joint': goal_pose.pose.position.x,
              u'odom_y_joint': goal_pose.pose.position.y,
              u'odom_z_joint': rotation_from_matrix(quaternion_matrix([goal_pose.pose.orientation.x,
                                                                       goal_pose.pose.orientation.y,
                                                                       goal_pose.pose.orientation.z,
                                                                       goal_pose.pose.orientation.w]))[0]}
        self.allow_all_collisions()
        self.send_and_check_joint_goal(js)


class Boxy(GiskardTestWrapper):
    def __init__(self):
        self.camera_tip = u'camera_link'
        self.r_tip = u'right_gripper_tool_frame'
        self.l_tip = u'left_gripper_tool_frame'
        super(Boxy, self).__init__(u'boxy_sim.yaml')

    def move_base(self, goal_pose):
        goal_pose = transform_pose(self.default_root, goal_pose)
        js = {u'odom_x_joint': goal_pose.pose.position.x,
              u'odom_y_joint': goal_pose.pose.position.y,
              u'odom_z_joint': rotation_from_matrix(quaternion_matrix([goal_pose.pose.orientation.x,
                                                                       goal_pose.pose.orientation.y,
                                                                       goal_pose.pose.orientation.z,
                                                                       goal_pose.pose.orientation.w]))[0]}
        self.allow_all_collisions()
        self.send_and_check_joint_goal(js)


class HSR(GiskardTestWrapper):
    def __init__(self):
        self.tip = u'hand_palm_link'
        super(HSR, self).__init__(u'hsr.yaml')

    def move_base(self, goal_pose):
        goal_pose = transform_pose(self.default_root, goal_pose)
        js = {u'odom_x': goal_pose.pose.position.x,
              u'odom_y': goal_pose.pose.position.y,
              u'odom_t': rotation_from_matrix(quaternion_matrix([goal_pose.pose.orientation.x,
                                                                 goal_pose.pose.orientation.y,
                                                                 goal_pose.pose.orientation.z,
                                                                 goal_pose.pose.orientation.w]))[0]}
        self.allow_all_collisions()
        self.send_and_check_joint_goal(js)

    def open_gripper(self):
        js = {u'hand_l_spring_proximal_joint': 0.7,
              u'hand_r_spring_proximal_joint': 0.7}
        self.send_and_check_joint_goal(js)

    def close_gripper(self):
        js = {u'hand_l_spring_proximal_joint': 0,
              u'hand_r_spring_proximal_joint': 0}
        self.send_and_check_joint_goal(js)

    # def command_gripper(self, width):
    #     js = {u'hand_motor_joint': width}
    #     self.send_and_check_joint_goal(js)


def publish_marker_sphere(position, frame_id=u'map', radius=0.05, id_=0):
    m = Marker()
    m.action = m.ADD
    m.ns = u'debug'
    m.id = id_
    m.type = m.SPHERE
    m.header.frame_id = frame_id
    m.pose.position.x = position[0]
    m.pose.position.y = position[1]
    m.pose.position.z = position[2]
    m.color = ColorRGBA(1, 0, 0, 1)
    m.scale.x = radius
    m.scale.y = radius
    m.scale.z = radius

    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
    start = rospy.get_rostime()
    while pub.get_num_connections() < 1 and (rospy.get_rostime() - start).to_sec() < 2:
        # wait for a connection to publisher
        # you can do whatever you like here or simply do nothing
        pass

    pub.publish(m)


def publish_marker_vector(start, end, diameter_shaft=0.01, diameter_head=0.02, id_=0):
    """
    assumes points to be in frame map
    :type start: Point
    :type end: Point
    :type diameter_shaft: float
    :type diameter_head: float
    :type id_: int
    """
    m = Marker()
    m.action = m.ADD
    m.ns = u'debug'
    m.id = id_
    m.type = m.ARROW
    m.points.append(start)
    m.points.append(end)
    m.color = ColorRGBA(1, 0, 0, 1)
    m.scale.x = diameter_shaft
    m.scale.y = diameter_head
    m.scale.z = 0
    m.header.frame_id = u'map'

    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
    start = rospy.get_rostime()
    while pub.get_num_connections() < 1 and (rospy.get_rostime() - start).to_sec() < 2:
        # wait for a connection to publisher
        # you can do whatever you like here or simply do nothing
        pass
    rospy.sleep(0.3)

    pub.publish(m)
