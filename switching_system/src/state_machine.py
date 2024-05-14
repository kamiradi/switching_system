#!/usr/bin/python
import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg as geom_msg
import sensor_msgs.msg as sense_msg
import control_msgs.msg as ctrl_msg
from pydrake.all import (
    RigidTransform, Quaternion, RollPitchYaw
)


def _write_pose_msg(X_AB, p, q):
    # p - position message
    # q - quaternion message
    X_AB = RigidTransform(X_AB)
    p.x, p.y, p.z = X_AB.translation()
    q.w, q.x, q.y, q.z = X_AB.rotation().ToQuaternion().wxyz()


def to_ros_transform(X_AB):
    """Converts Drake transform to ROS transform."""
    msg = geom_msg.Transform()
    _write_pose_msg(X_AB, p=msg.translation, q=msg.rotation)
    return msg


def to_ros_pose(X_AB):
    """Converts Drake transform to ROS pose."""
    msg = geom_msg.Pose()
    _write_pose_msg(X_AB, p=msg.position, q=msg.orientation)
    return msg


class FrankaStateMachine(object):
    """
    This class abstracts a motion planner in end-effector cartesian coordinates
    """

    def __init__(self, name, debug=True):
        # class related variables
        self._name = name

        # subscribers and pulishers
        self._joy_sub = rospy.Subscriber(
            "/spacenav/joy",
            sense_msg.Joy,
            self._joyCallback
        )
        self._equilibrium_pose_pub = rospy.Publisher(
            "/cartesian_impedance_controller/equilibrium_pose",
            geom_msg.PoseStamped,
            queue_size=1
        )
        self._gripper_cmd_pub = rospy.Publisher(
            "/gripper_controller/gripper_cmd/goal",
            ctrl_msg.GripperCommandActionGoal,
            queue_size=1
        )

        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
        self._tfBr = tf2_ros.TransformBroadcaster()

        rospy.loginfo("State Machine initialized")

    def _joyCallback(self, msg):
        button1 = msg.buttons[0]
        button2 = msg.buttons[1]
