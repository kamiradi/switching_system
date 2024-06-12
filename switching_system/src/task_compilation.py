import rospy
from enum import Enum
import math
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


class PlannerState(Enum):
    APPROACH = 1
    ALIGN = 2
    INSERT = 3
    INITIALISE = 4
    PRISMATIC = 5
    FAILURE = 6
    RESET = 7
    TELEOP = 8
    TERMINATE = 9
    GRASP = 10
    PREGRASP = 11
    GRASPPROACH = 12
    GRASPREL = 13  # releases grasp


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

    X_Ggrasp = from_ros_pose(pose_fence['X_Pregrasp'].pose.pose) @ RigidTransform(
        [0, 0, 0.03])
    X_Ginsert = from_ros_pose(pose_fence['X_Preinsert'].pose.pose) @ RigidTransform(
        [0, 0, 0.03]
    )
    pose_fence['X_Ggrasp'] = CmdPoseGoal()
    pose_fence['X_Ggrasp'].pose.pose = to_ros_pose(X_Ggrasp)
    pose_fence['X_Ggrasp'].pose.use_estimation = False
    pose_fence['X_Ginsert'] = CmdPoseGoal()
    pose_fence['X_Ginsert'].pose.pose = to_ros_pose(X_Ginsert)
    pose_fence['X_Ginsert'].pose.use_estimation = False

    return pose_fence


def lissajous(A, B, a, b, delta, step):
    """
    delta in radians
    """
    x = A * math.sin(a * step + delta)
    y = B * math.sin(b * step)
    return (x, y)


def getLissajousPoseFence(estimation=True):
    pose_fence = {}
    prismatic = 0.0375
    lissajous_params = (0.04, 0.04, 0.02, 0.03, -np.pi/2)

    pose_fence['X_Pregrasp'] = keyframe_to_pose("/keyframes/X_Pregrasp")
    pose_fence['X_Gstart'] = keyframe_to_pose("/keyframes/X_Gstart")
    pose_fence['X_Insertapproach'] = keyframe_to_pose(
        "/keyframes/X_Insertapproach")
    pose_fence['X_Preinsert'] = keyframe_to_pose("/keyframes/X_Preinsert")

    # increase height of pre-insert
    X_Preinsert = from_ros_pose(pose_fence['X_Preinsert'].pose.pose) @ RigidTransform(
        [0, 0, -0.04]
    )
    pose_fence['X_Preinsert'] = CmdPoseGoal()
    pose_fence['X_Preinsert'].pose.pose = to_ros_pose(X_Preinsert)
    pose_fence['X_Preinsert'].pose.use_estimation = False

    # Grasp keypoint
    X_Ggrasp = from_ros_pose(pose_fence['X_Pregrasp'].pose.pose) @ RigidTransform(
        [0, 0, 0.02])
    pose_fence['X_Ggrasp'] = CmdPoseGoal()
    pose_fence['X_Ggrasp'].pose.pose = to_ros_pose(X_Ggrasp)
    pose_fence['X_Ggrasp'].pose.use_estimation = False

    for i in range(30):
        x, y = lissajous(*lissajous_params, i*50)
        X_temp = X_Preinsert @ RigidTransform(
            [x, y, 0.0]
        )
        X_Ginsert = X_temp @ RigidTransform(
            [0, 0, prismatic]
        )
        pose_fence['X_Preinsert_{}'.format(i)] = CmdPoseGoal()
        pose_fence['X_Preinsert_{}'.format(i)].pose.pose = to_ros_pose(X_temp)
        pose_fence['X_Preinsert_{}'.format(i)].pose.use_estimation = False

        pose_fence['X_Ginsert_{}'.format(i)] = CmdPoseGoal()
        pose_fence['X_Ginsert_{}'.format(i)].pose.pose = to_ros_pose(X_Ginsert)
        pose_fence['X_Ginsert_{}'.format(i)].pose.use_estimation = estimation

    return pose_fence


def getPrismaticPoseFence():
    """
    Retrieves keyframes relavant to the prismatic experiment and 
    """
    pose_fence = {}
    prismatic = 0.04

    pose_fence['X_Pregrasp'] = keyframe_to_pose("/keyframes/X_Pregrasp")
    pose_fence['X_Gstart'] = keyframe_to_pose("/keyframes/X_Gstart")
    pose_fence['X_Insertapproach'] = keyframe_to_pose(
        "/keyframes/X_Insertapproach")
    pose_fence['X_Preinsert'] = keyframe_to_pose("/keyframes/X_Preinsert")

    # increase height of pre-insert
    X_Preinsert = from_ros_pose(pose_fence['X_Preinsert'].pose.pose) @ RigidTransform(
        [0, 0, -0.01]
    )
    pose_fence['X_Preinsert'] = CmdPoseGoal()
    pose_fence['X_Preinsert'].pose.pose = to_ros_pose(X_Preinsert)
    pose_fence['X_Preinsert'].pose.use_estimation = False

    X_Ggrasp = from_ros_pose(pose_fence['X_Pregrasp'].pose.pose) @ RigidTransform(
        [0, 0, 0.02])
    X_Ginsert = from_ros_pose(pose_fence['X_Preinsert'].pose.pose) @ RigidTransform(
        [0, 0, prismatic]
    )
    pose_fence['X_Ggrasp'] = CmdPoseGoal()
    pose_fence['X_Ggrasp'].pose.pose = to_ros_pose(X_Ggrasp)
    pose_fence['X_Ggrasp'].pose.use_estimation = False
    pose_fence['X_Ginsert'] = CmdPoseGoal()
    pose_fence['X_Ginsert'].pose.pose = to_ros_pose(X_Ginsert)
    pose_fence['X_Ginsert'].pose.use_estimation = True

    return pose_fence


##### State Machine Creation #####


def createGraphicalStateMachine():
    """
    Creates a directed graph to be used by the Behaviour Tree. This converts
    the Behaviour Tree into a Finite State Machine.
    """
    sm = nx.DiGraph()
    root = State()
    pre_grasp = State(
        PlannerState.GRASPPROACH,
        status=Status.INACTIVE
    )
    sm.add_edge(root, pre_grasp, transition="SUCCESS")
    sm.add_edge(pre_grasp, root, transition="SUCCESS")

    return sm


def createGraspStateMachine(pose_fence):
    """
    Gets a dictionary of pose waypoints, returns a queue of waypoints and
    corresponding states
    """
    waypoints = queue.Queue(maxsize=15)
    modes = queue.Queue(maxsize=15)

    waypoints.put(pose_fence['X_Gstart'])
    modes.put(PlannerState.INITIALISE)

    waypoints.put(pose_fence['X_Pregrasp'])
    modes.put(PlannerState.GRASPPROACH)

    waypoints.put(pose_fence['X_Ggrasp'])
    modes.put(PlannerState.PREGRASP)

    waypoints.put(pose_fence['X_Ggrasp'])
    modes.put(PlannerState.GRASP)

    waypoints.put(pose_fence['X_Insertapproach'])
    modes.put(PlannerState.APPROACH)

    waypoints.put(pose_fence['X_Preinsert'])
    modes.put(PlannerState.ALIGN)

    waypoints.put(pose_fence['X_Ginsert'])
    modes.put(PlannerState.INSERT)

    waypoints.put(pose_fence['X_Ginsert'])
    modes.put(PlannerState.GRASPREL)

    waypoints.put(pose_fence['X_Gstart'])
    modes.put(PlannerState.TERMINATE)

    return waypoints, modes


def createLissajousStateMachine(pose_fence):
    """
    Takes the pre-insert pose + takes time step -> adds next point in lissajous
    curve
    """
    waypoints = queue.Queue(maxsize=500)
    modes = queue.Queue(maxsize=500)

    waypoints.put(pose_fence['X_Gstart'])
    modes.put(PlannerState.INITIALISE)

    waypoints.put(pose_fence['X_Pregrasp'])
    modes.put(PlannerState.GRASPPROACH)

    waypoints.put(pose_fence['X_Ggrasp'])
    modes.put(PlannerState.PREGRASP)

    waypoints.put(pose_fence['X_Ggrasp'])
    modes.put(PlannerState.GRASP)

    waypoints.put(pose_fence['X_Insertapproach'])
    modes.put(PlannerState.APPROACH)

    # waypoints.put(pose_fence['X_Preinsert'])
    # modes.put(PlannerState.ALIGN)

    for i in range(30):
        waypoints.put(pose_fence['X_Preinsert_{}'.format(i)])
        modes.put(PlannerState.ALIGN)

        waypoints.put(pose_fence['X_Ginsert_{}'.format(i)])
        modes.put(PlannerState.INSERT)

        waypoints.put(pose_fence['X_Preinsert_{}'.format(i)])
        modes.put(PlannerState.ALIGN)

    waypoints.put(pose_fence['X_Preinsert_29'])
    modes.put(PlannerState.GRASPREL)

    waypoints.put(pose_fence['X_Gstart'])
    modes.put(PlannerState.TERMINATE)

    return waypoints, modes


def createEstimationStateMachine(pose_fence):
    """
    Gets a dictionary of pose waypoints, returns a queue of waypoints and
    corresponding states
    """
    waypoints = queue.Queue(maxsize=50)
    modes = queue.Queue(maxsize=50)

    waypoints.put(pose_fence['X_Gstart'])
    modes.put(PlannerState.INITIALISE)

    waypoints.put(pose_fence['X_Preinsert'])
    modes.put(PlannerState.ALIGN)

    waypoints.put(pose_fence['X_Preinsert'])
    modes.put(PlannerState.GRASP)

    waypoints.put(pose_fence['X_Ginsert'])
    modes.put(PlannerState.INSERT)

    waypoints.put(pose_fence['X_Preinsert'])
    modes.put(PlannerState.ALIGN)

    waypoints.put(pose_fence['X_Ginsert'])
    modes.put(PlannerState.INSERT)

    waypoints.put(pose_fence['X_Preinsert'])
    modes.put(PlannerState.ALIGN)

    waypoints.put(pose_fence['X_Ginsert'])
    modes.put(PlannerState.INSERT)

    waypoints.put(pose_fence['X_Preinsert'])
    modes.put(PlannerState.ALIGN)

    waypoints.put(pose_fence['X_Ginsert'])
    modes.put(PlannerState.INSERT)

    waypoints.put(pose_fence['X_Preinsert'])
    modes.put(PlannerState.ALIGN)

    waypoints.put(pose_fence['X_Ginsert'])
    modes.put(PlannerState.INSERT)

    waypoints.put(pose_fence['X_Preinsert'])
    modes.put(PlannerState.ALIGN)

    waypoints.put(pose_fence['X_Ginsert'])
    modes.put(PlannerState.INSERT)

    waypoints.put(pose_fence['X_Preinsert'])
    modes.put(PlannerState.GRASPREL)

    waypoints.put(pose_fence['X_Gstart'])
    modes.put(PlannerState.TERMINATE)

    return waypoints, modes


def createDebugStateMachine(pose_fence):
    """
    Gets a dictionary of poses, returns a queue of waypoints and corresponding
    discrete states
    """
    waypoints = queue.Queue(maxsize=5)
    modes = queue.Queue(maxsize=5)

    waypoints.put(pose_fence['X_Gstart'])
    modes.put(PlannerState.INITIALISE)

    waypoints.put(pose_fence['X_Pregrasp'])
    modes.put(PlannerState.PREGRASP)

    waypoints.put(pose_fence['X_Gstart'])
    modes.put(PlannerState.RESET)

    return waypoints, modes
