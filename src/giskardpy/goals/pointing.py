from __future__ import division

from typing import Optional

from geometry_msgs.msg import Vector3Stamped, PointStamped

from giskardpy import casadi_wrapper as w
from giskardpy.goals.goal import Goal, WEIGHT_BELOW_CA, WEIGHT_ABOVE_CA
import giskardpy.utils.tfwrapper as tf
from giskardpy.utils.logging import logwarn


class Pointing(Goal):
    def __init__(self,
                 tip_link: str,
                 goal_point: PointStamped,
                 root_link: str,
                 tip_group: Optional[str] = None,
                 root_group: Optional[str] = None,
                 pointing_axis: Vector3Stamped = None,
                 max_velocity: float = 0.3,
                 weight: float = WEIGHT_BELOW_CA,
                 **kwargs):
        """
        Will orient pointing_axis at goal_point.
        :param tip_link: tip link of the kinematic chain.
        :param goal_point: where to point pointing_axis at.
        :param root_link: root link of the kinematic chain.
        :param tip_group: if tip_link is not unique, search this group for matches.
        :param root_group: if root_link is not unique, search this group for matches.
        :param pointing_axis: the axis of tip_link that will be used for pointing
        :param max_velocity: rad/s
        :param weight:
        """
        super().__init__(**kwargs)
        self.weight = weight
        self.max_velocity = max_velocity
        self.root = self.world.get_link_name(root_link, root_group)
        self.tip = self.world.get_link_name(tip_link, tip_group)
        self.root_P_goal_point = self.transform_msg(self.root, goal_point)

        if pointing_axis is not None:
            self.tip_V_pointing_axis = self.transform_msg(self.tip, pointing_axis)
            self.tip_V_pointing_axis.vector = tf.normalize(self.tip_V_pointing_axis.vector)
        else:
            logwarn(f'Deprecated warning: Please set pointing_axis.')
            self.tip_V_pointing_axis = Vector3Stamped()
            self.tip_V_pointing_axis.header.frame_id = self.tip
            self.tip_V_pointing_axis.vector.z = 1

    def make_constraints(self):
        root_T_tip = self.get_fk(self.root, self.tip)
        root_P_goal_point = w.ros_msg_to_matrix(self.root_P_goal_point)
        tip_V_pointing_axis = w.ros_msg_to_matrix(self.tip_V_pointing_axis)

        root_V_goal_axis = root_P_goal_point - w.position_of(root_T_tip)
        root_V_goal_axis = w.scale(root_V_goal_axis, 1)
        root_V_pointing_axis = w.dot(root_T_tip, tip_V_pointing_axis)

        self.add_vector_goal_constraints(frame_V_current=root_V_pointing_axis,
                                         frame_V_goal=root_V_goal_axis,
                                         reference_velocity=self.max_velocity,
                                         weight=self.weight)

    def __str__(self):
        s = super().__str__()
        return f'{s}/{self.root}/{self.tip}'
