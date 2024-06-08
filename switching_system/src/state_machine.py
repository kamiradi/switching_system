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
from behaviours import *
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


def keyframe_to_pose(keyframe):
    """
    Converts keyframe in ros param to CmdPoseGoal object
    """
    pose_goal = CmdPoseGoal()
    pose = rospy.get_param(keyframe)

    pose_goal.pose.pose.position.x = pose["xyz"][0]
    pose_goal.pose.pose.position.y = pose["xyz"][1]
    pose_goal.pose.pose.position.z = pose["xyz"][2]
    pose_goal.pose.pose.orientation.w = pose["wxyz"][0]
    pose_goal.pose.pose.orientation.x = pose["wxyz"][1]
    pose_goal.pose.pose.orientation.y = pose["wxyz"][2]
    pose_goal.pose.pose.orientation.z = pose["wxyz"][3]
    pose_goal.pose.use_estimation = False

    return pose_goal


def getInsertionPoseFence():
    pose_fence = {}

    pose_fence['X_Pregrasp'] = keyframe_to_pose("/keyframes/X_Pregrasp")
    pose_fence['X_Gstart'] = keyframe_to_pose("/keyframes/X_Gstart")
    pose_fence['X_Insertapproach'] = keyframe_to_pose(
        "/keyframes/X_Insertapproach")
    pose_fence['X_Preinsert'] = keyframe_to_pose("/keyframes/X_Preinsert")

    X_Ggrasp = from_ros_pose(pose_fence['X_Pregrasp'].pose) @ RigidTransform(
        [0, 0, 0.03])
    X_Ginsert = from_ros_pose(pose_fence['X_Preinsert'].pose) @ RigidTransform(
        [0, 0, 0.03]
    )
    pose_fence['X_Ggrasp'] = CmdPoseGoal()
    pose_fence['X_Ggrasp'].pose.pose = to_ros_pose(X_Ggrasp)
    pose_fence['X_Ggrasp'].pose.use_estimation = False
    pose_fence['X_Ginsert'] = CmdPoseGoal()
    pose_fence['X_Ginsert'].pose.pose = to_ros_pose(X_Ginsert)
    pose_fence['X_Ginsert'].pose.use_estimation = False

    return pose_fence


def getLissajousePoseFence():
    pose_fence = {}
    return pose_fence


def getPrismaticPoseFence():
    """
    Retrieves keyframes relavant to the prismatic experiment and 
    """
    pose_fence = {}
    prismatic = 0.02

    pose_fence['X_Pregrasp'] = keyframe_to_pose("/keyframes/X_Pregrasp")
    pose_fence['X_Gstart'] = keyframe_to_pose("/keyframes/X_Gstart")
    pose_fence['X_Insertapproach'] = keyframe_to_pose(
        "/keyframes/X_Insertapproach")
    pose_fence['X_Preinsert'] = keyframe_to_pose("/keyframes/X_Preinsert")

    # increase height of pre-insert
    X_Preinsert = from_ros_pose(pose_fence['X_Preinsert'].pose) @ RigidTransform(
        [0, 0, -0.01]
    )
    pose_fence['X_Preinsert'] = CmdPoseGoal()
    pose_fence['X_Preinsert'].pose = to_ros_pose(X_Preinsert)

    X_Ggrasp = from_ros_pose(pose_fence['X_Pregrasp'].pose) @ RigidTransform(
        [0, 0, 0.02])
    X_Ginsert = from_ros_pose(pose_fence['X_Preinsert'].pose) @ RigidTransform(
        [0, 0, prismatic]
    )
    pose_fence['X_Ggrasp'] = CmdPoseGoal()
    pose_fence['X_Ggrasp'].pose = to_ros_pose(X_Ggrasp)
    pose_fence['X_Ginsert'] = CmdPoseGoal()
    pose_fence['X_Ginsert'].pose = to_ros_pose(X_Ginsert)

    return pose_fence


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


def MakePegPoseTrajectory(X_G, times):
    sample_times = []
    poses = []
    for name in ["initial", "final"]:
        sample_times.append(times[name])
        poses.append(X_G[name])

    return PiecewisePose.MakeLinear(sample_times, poses)


class FrankaDebugStateMachine(object):
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

        self.action_client1 = actionlib.SimpleActionClient(
            'cmd_pose',
            CmdPoseAction
        )
        self.action_client2 = actionlib.SimpleActionClient(
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
        self.pose_fence = getPoseFence()
        self.goal_cmd_pose1 = self.pose_fence['X_Gstart']
        self.goal_cmd_pose2 = self.pose_fence['X_Pregrasp']
        self.goal_cmd_pose3 = self.pose_fence['X_Gstart']

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
        self.action_client1.wait_for_server()
        self.action_client1.send_goal(self.goal_cmd_pose1)
        # rospy.sleep(2)
        self.action_client2.wait_for_server()
        self.action_client2.send_goal_and_wait(self.goal_cmd_pose2)
        self.action_client1.send_goal_and_wait(self.goal_cmd_pose3)
        result = self.action_client2.get_result()
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

        self.pose_fence = getInsertionPoseFence()

        # setup state machine
        # self.state_machine = constructBT(self.pose_fence)
        self.state_machine = GraspBT(self.pose_fence)
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


class FrankaEstimationMachine(object):
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

        self.pose_fence = getPrismaticPoseFence()

        # setup state machine
        # self.state_machine = constructBT(self.pose_fence)
        self.state_machine = GraspBT(self.pose_fence)
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


def main():

    # do something
    pose_fence = getPoseFence()
    action1 = SimpleActionClient(
        'cmd_pose',
        CmdPoseAction
    )
    action2 = SimpleActionClient(
        'cmd_pose',
        CmdPoseAction
    )
    return


if __name__ == "__main__":
    rospy.init_node('state_machine', anonymous=False)
    state_machine = FrankaEstimationMachine(rospy.get_name(), False)
    # state_machine = FrankaBTStateMachine(rospy.get_name(), False)
    # state_machine = FrankaDebugStateMachine(rospy.get_name(), False)
    rospy.sleep(3)
    state_machine.execute()
    rospy.spin()
