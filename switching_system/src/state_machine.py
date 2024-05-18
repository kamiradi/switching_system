#!/usr/bin/python
import rospy
import actionlib
import sys
import functools
import tf2_ros
import numpy as np
import geometry_msgs.msg as geom_msg
import sensor_msgs.msg as sense_msg
import control_msgs.msg as ctrl_msg
from switching_system.msg import (
    CmdPoseAction, CmdPoseFeedback, CmdPoseResult, CmdPoseGoal
)
from pydrake.all import (
    RigidTransform, Quaternion, RollPitchYaw, PiecewisePose
)
from states import *
import py_trees_ros
import py_trees.console as console


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


def _read_pose_msg(p, q):
    # p - position message
    # q - quaternion message
    return RigidTransform(
        Quaternion(wxyz=[q.w, q.x, q.y, q.z]), [p.x, p.y, p.z])


def from_ros_pose(pose):
    """Converts ROS pose to Drake transform."""
    return _read_pose_msg(p=pose.position, q=pose.orientation)


def from_ros_transform(tr):
    """Converts ROS transform to Drake transform."""
    return _read_pose_msg(p=tr.translation, q=tr.rotation)


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


def MakePegPoseTrajectory(X_G, times):
    sample_times = []
    poses = []
    for name in ["initial", "final"]:
        sample_times.append(times[name])
        poses.append(X_G[name])

    return PiecewisePose.MakeLinear(sample_times, poses)


class FrankaStateMachine(object):
    """
    This class abstracts a motion planner in end-effector cartesian coordinates
    """

    def __init__(self, name, debug=True):

        # class related variables
        self._name = name
        self._freq = 100
        self._timer = None
        self._idx = 0
        self._steps = 0
        self._velocity = 0.05
        self._plan = None
        self._plan_start_time = None
        self._debug = debug

        # subscribers and pulishers
        self._joy_sub = rospy.Subscriber(
            "/spacenav/joy",
            sense_msg.Joy,
            self._joyCallback
        )
        self._gripper_cmd_pub = rospy.Publisher(
            "/gripper_controller/gripper_cmd/goal",
            ctrl_msg.GripperCommandActionGoal,
            queue_size=1
        )

        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
        self._tfBr = tf2_ros.TransformBroadcaster()

        self.action_client = actionlib.SimpleActionClient(
            'cmd_pose',
            CmdPoseAction
        )
        self.goal_cmd_pose = CmdPoseGoal()
        self._tf_X_Gstart = None

        rospy.loginfo("State Machine initialized")

    def _joyCallback(self, msg):
        button1 = msg.buttons[0]
        button2 = msg.buttons[1]

    def execute(self):
        # get all the keyframes
        pregrasp = rospy.get_param("/keyframes/X_Pregrasp")
        start = rospy.get_param("/keyframes/X_Gstart")
        self.goal_cmd_pose.pose.position.x = pregrasp["xyz"][0]
        self.goal_cmd_pose.pose.position.y = pregrasp["xyz"][1]
        self.goal_cmd_pose.pose.position.z = pregrasp["xyz"][2]
        self.goal_cmd_pose.pose.orientation.w = pregrasp["wxyz"][0]
        self.goal_cmd_pose.pose.orientation.x = pregrasp["wxyz"][1]
        self.goal_cmd_pose.pose.orientation.y = pregrasp["wxyz"][2]
        self.goal_cmd_pose.pose.orientation.z = pregrasp["wxyz"][3]

        # get current pose
        try:
            tf_X_EE = self._tfBuffer.lookup_transform(
                'panda_link0',
                'panda_EE',
                rospy.Time())
            self._tf_X_Gstart = tf_X_EE

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return

        dur = rospy.Duration(1.0/self._freq)
        self._timer = rospy.Timer(dur, self.timer_callback)
        rospy.loginfo("Starting timer events")
        # self._steps = horizon * self._freq

    def timer_callback(self, event):
        rospy.loginfo("Timer event")
        self.action_client.wait_for_server()
        self.action_client.send_goal_and_wait(self.goal_cmd_pose)
        result = self.action_client.get_result()
        self.reachedGoal = result.reached_goal


class FrankaBTStateMachine(object):
    """
    This class abstracts a motion planner in end-effector cartesian coordinates
    """

    def __init__(self, name, debug=True):

        # class related variables
        self._name = name
        self._freq = 100
        self._timer = None
        self._idx = 0
        self._steps = 0
        self._velocity = 0.05
        self._plan = None
        self._plan_start_time = None
        self._debug = debug

        # subscribers and pulishers
        self._joy_sub = rospy.Subscriber(
            "/spacenav/joy",
            sense_msg.Joy,
            self._joyCallback
        )
        self._gripper_cmd_pub = rospy.Publisher(
            "/gripper_controller/gripper_cmd/goal",
            ctrl_msg.GripperCommandActionGoal,
            queue_size=1
        )

        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
        self._tfBr = tf2_ros.TransformBroadcaster()

        self.action_client = actionlib.SimpleActionClient(
            'cmd_pose',
            CmdPoseAction
        )
        self.goal_cmd_pose = CmdPoseGoal()
        self._tf_X_Gstart = None

        rospy.loginfo("State Machine initialized")

    def _joyCallback(self, msg):
        button1 = msg.buttons[0]
        button2 = msg.buttons[1]

    def execute(self):
        # get all the keyframes
        pregrasp = rospy.get_param("/keyframes/X_Pregrasp")
        start = rospy.get_param("/keyframes/X_Gstart")
        X_Pregrasp = CmdPoseGoal()
        X_Gstart = CmdPoseGoal()

        X_Pregrasp.pose.position.x = pregrasp["xyz"][0]
        X_Pregrasp.pose.position.y = pregrasp["xyz"][1]
        X_Pregrasp.pose.position.z = pregrasp["xyz"][2]
        X_Pregrasp.pose.orientation.w = pregrasp["wxyz"][0]
        X_Pregrasp.pose.orientation.x = pregrasp["wxyz"][1]
        X_Pregrasp.pose.orientation.y = pregrasp["wxyz"][2]
        X_Pregrasp.pose.orientation.z = pregrasp["wxyz"][3]

        try:
            tf_X_EE = self._tfBuffer.lookup_transform(
                'panda_link0',
                'panda_EE',
                rospy.Time())
            self._tf_X_Gstart = tf_X_EE
            # X_Gstart_drake = from_ros_transform(tf_X_EE.transform)
            # X_del = RigidTransform([0, 0, 0.03])
            # X_Gstart_new = X_Gstart_drake @ X_del
            # X_Gstart.pose = to_ros_pose(X_Gstart_new)

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return

        X_Gstart.pose.position.x = start["xyz"][0]
        X_Gstart.pose.position.y = start["xyz"][1]
        X_Gstart.pose.position.z = start["xyz"][2]
        X_Gstart.pose.orientation.w = start["wxyz"][0]
        X_Gstart.pose.orientation.x = start["wxyz"][1]
        X_Gstart.pose.orientation.y = start["wxyz"][2]
        X_Gstart.pose.orientation.z = start["wxyz"][3]

        self.pose_fence = {}
        self.pose_fence['X_Pregrasp'] = X_Pregrasp
        self.pose_fence['X_Gstart'] = X_Gstart

        # setup state machine
        self.state_machine = constructBT(self.pose_fence)
        behaviour_tree = py_trees_ros.trees.BehaviourTree(self.state_machine)
        # self.state_machine.setup()
        # get current pose
        try:
            tf_X_EE = self._tfBuffer.lookup_transform(
                'panda_link0',
                'panda_EE',
                rospy.Time())
            self._tf_X_Gstart = tf_X_EE

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return

        rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
        if not behaviour_tree.setup(timeout=15):
            console.logerror("failed to setup tree, aborting.")
            sys.exit(1)
        behaviour_tree.tick_tock(500)
        # dur = rospy.Duration(1.0/self._freq)
        # self._timer = rospy.Timer(dur, self.timer_callback)
        # rospy.loginfo("Starting timer events")
        # self._steps = horizon * self._freq

    def timer_callback(self, event):
        # rospy.loginfo("Timer event")
        # self.action_client.wait_for_server()
        # self.action_client.send_goal_and_wait(self.goal_cmd_pose)
        # result = self.action_client.get_result()
        # self.reachedGoal = result.reached_goal
        try:
            self.state_machine.tick()
        except KeyboardInterrupt():
            self._timer.shutdown()


if __name__ == "__main__":
    rospy.init_node('StateMachine', anonymous=True)
    state_machine = FrankaBTStateMachine(rospy.get_name(), False)
    rospy.sleep(3)
    state_machine.execute()
    rospy.spin()
