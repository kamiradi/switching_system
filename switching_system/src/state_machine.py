#!/usr/bin/python
import rospy
import actionlib
import sys
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
        grasp = rospy.get_param("/keyframes/X_Pregrasp")
        # X_Pregrasp = RigidTransform(
        #     Quaternion(wxyz=np.array(grasp["wxyz"])),
        #     grasp["xyz"]
        # )
        self.goal_cmd_pose.pose.position.x = grasp["xyz"][0]
        self.goal_cmd_pose.pose.position.y = grasp["xyz"][1]
        self.goal_cmd_pose.pose.position.z = grasp["xyz"][2]
        self.goal_cmd_pose.pose.orientation.w = grasp["wxyz"][0]
        self.goal_cmd_pose.pose.orientation.x = grasp["wxyz"][1]
        self.goal_cmd_pose.pose.orientation.y = grasp["wxyz"][2]
        self.goal_cmd_pose.pose.orientation.z = grasp["wxyz"][3]

        # get current pose
        try:
            tf_X_EE = self._tfBuffer.lookup_transform(
                'panda_link0',
                'panda_EE',
                rospy.Time())
            self._tf_X_Gstart = tf_X_EE
            # X_Gcurr = from_ros_transform(tf_X_EE.transform)
            # # rospy.loginfo("drake translation: {}".format(
            # #     X_Pregrasp.translation()))
            # # rospy.loginfo("tf translation: {}".format(
            # #     np.array([t.x, t.y, t.z])))
            # horizon = np.linalg.norm(
            #     X_Pregrasp.translation() -
            #     X_Gcurr.translation()) / self._velocity
            # rospy.loginfo("time horizon: {}".format(horizon))
            # X_G = {}
            # times = {}
            # X_G["initial"] = X_Gcurr
            # X_G["final"] = X_Pregrasp
            # times["initial"] = 0.0
            # times["final"] = horizon
            # self._plan = MakePegPoseTrajectory(X_G, times)

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return

        dur = rospy.Duration(1.0/self._freq)
        self._timer = rospy.Timer(dur, self.timer_callback)
        rospy.loginfo("Starting timer events")
        # self._steps = horizon * self._freq

    def timer_callback(self, event):
        # rospy.loginfo("timer ticking: {}".format(event))
        # if self._plan is None:
        #     rospy.loginfo("Motion plan has not been set!")
        #     self._timer.shutdown()
        # if self._idx == 0:
        #     self._plan_start_time = rospy.Time.now().to_sec()
        # if self._idx < self._steps:
        #     self._idx += 1
        #     time = rospy.Time.now().to_sec() - self._plan_start_time
        #     X_Gcommand = self._plan.GetPose(time)

        #     if not self._debug:
        #         # sends calculated pose to robot
        #         pose = geom_msg.PoseStamped()
        #         pose.header.stamp = rospy.Time.now()
        #         pose.header.frame_id = "e_pose"
        #         pose.pose = to_ros_pose(X_Gcommand)

        #         self._equilibrium_pose_pub.publish(pose)
        # else:
        #     self._timer.shutdown()
        #     rospy.loginfo("Plan succeeded!")
        #     return
        rospy.loginfo("Timer event")
        self.action_client.wait_for_server()
        self.action_client.send_goal_and_wait(self.goal_cmd_pose)
        result = self.action_client.get_result()
        self.reachedGoal = result.reached_goal


if __name__ == "__main__":
    rospy.init_node('StateMachine', anonymous=True)
    state_machine = FrankaStateMachine(rospy.get_name(), False)
    rospy.sleep(3)
    state_machine.execute()
    rospy.spin()
