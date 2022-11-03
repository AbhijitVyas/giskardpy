#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA, String
from tf2_msgs.msg import TFMessage
from datetime import datetime
from tf import TransformListener
import tf2_py
import numpy as np


RED_COLOR   = ColorRGBA(255.0, 0.0, 0.0, 0.8)
MAROON_COLOR= ColorRGBA(128, 0, 0, 0.8)
GREEN_COLOR = ColorRGBA(0, 2, 0, 0.8)
YELLOW_COLOR= ColorRGBA(255.0, 255.0, 0, 0.8)
OLIVE_COLOR = ColorRGBA(128.0, 128.0, 0, 0.8)
AQUA_COLOR  = ColorRGBA(0.0, 255.0, 255.0, 0.8)
fuchsia_COLOR  = ColorRGBA(255.0, 0.0, 255.0, 0.8)
BLUE_COLOR  = ColorRGBA(0.0, 0.0, 255.0, 0.8)
purple_COLOR = ColorRGBA(128.0, 0.0, 128.0, 0.8)
BLACK_COLOR = ColorRGBA(0.0, 0.0, 0.0, 0.8)

COLORs = [RED_COLOR, MAROON_COLOR, GREEN_COLOR, YELLOW_COLOR, OLIVE_COLOR,
          AQUA_COLOR, fuchsia_COLOR, BLUE_COLOR, purple_COLOR, BLACK_COLOR]

trajectoryPoints = np.array([])
def returnTrajectoryPointVector():
    return trajectoryPoints

class KnowRobTrajectoryFollower:
    def __init__(self):
        self.marker = None
        self.count = 0
        self.points = np.array([])

        print("I am trying to listen: ")
        listener = TransformListener()
        # this controls the rate at which we listen to tf messages per second
        rate = rospy.Rate(1)  # 10hz
        while not rospy.is_shutdown():
            try:
                (data, rot) = listener.lookupTransform('/map', '/SK_GenesisRightHand', rospy.Time(0))
                self.points = np.append(self.points, data)
                trajectoryPoints = self.points
                self.show_points_in_rviz(data)
                rate.sleep()
            except tf2_py.ConnectivityException:
                rospy.logerr("ConnectivityException happened")
            except tf2_py.LookupException:
                rospy.logerr("LookupException happened")
            except tf2_py.ExtrapolationException:
                rospy.logerr("ExtrapolationException happened")
            except tf2_py.InvalidArgumentException:
                rospy.logerr("InvalidArgumentException happened")


    
    

    def show_points_in_rviz(self, data):
        self.marker = Marker()
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        # first check time stamp values and assign color based upon action time stamps
        self.timeStamp = rospy.Time()
        MARKERCOLOR = GREEN_COLOR
        self.marker = Marker(
            type=Marker.SPHERE,
            id=0,
            lifetime=rospy.Duration(1000),
            pose=Pose(Point(data[0],
                            data[1],
                            data[2]),
                      Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.01, 0.01, 0.01),
            header=Header(frame_id='map'),
            color=MARKERCOLOR)
        self.count += 1
        self.marker.id = self.count
        self.marker_publisher.publish(self.marker)


if __name__ == '__main__':
    rospy.init_node("knowrobTrajectoryFollower", anonymous=True)
    trajectory_interactive_markers = KnowRobTrajectoryFollower()
    rospy.sleep(0.5)
    rospy.spin()