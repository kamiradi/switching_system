#!/usr/bin/python
import rospy
import actionlib
import sys
import functools
import tf2_ros
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
import task_compilation as tc


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
        self.state_machine = EstimationBT(self.pose_fence)
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


class LissajousEstimationMachine(object):
    """
    This class abstracts a motion planner in end-effector cartesian coordinates
    """

    def __init__(
            self,
            name,
            pose_fence_callback,
            state_machine_callback,
            debug=True):

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
        self._pose_fence_cb = pose_fence_callback
        self._sm_callback = state_machine_callback

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
            rospy.warn("Did not get transform")
            return

        self.pose_fence = self._pose_fence_cb(estimation=True)
        waypoints, modes = self._sm_callback(self.pose_fence)
        compiled_task = {}
        compiled_task['waypoints'] = waypoints
        compiled_task['modes'] = modes

        # setup state machine
        # self.state_machine = constructBT(self.pose_fence)
        self.state_machine = BT(compiled_task)
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

        try:
            behaviour_tree.tick_tock(500)
        except Exception as e:
            rospy.warn(e)
            rospy.warn("Shutting down behaviour tree ..")
            functools.partial(shutdown, behaviour_tree)
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
    state_machine = LissajousEstimationMachine(
        rospy.get_name(),
        pose_fence_callback=tc.getLissajousPoseFence,
        state_machine_callback=tc.createLissajousStateMachine,
        debug=False)
    # state_machine = FrankaEstimationMachine(rospy.get_name(), False)
    # state_machine = FrankaBTStateMachine(rospy.get_name(), False)
    # state_machine = FrankaDebugStateMachine(rospy.get_name(), False)
    rospy.sleep(2)
    state_machine.execute()
    rospy.spin()
