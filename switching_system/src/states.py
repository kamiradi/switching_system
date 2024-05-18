import numpy as np
import networkx as nx
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status
from py_trees.composites import Selector, Sequence
# from py_trees.decorators import Repeat
from py_trees_ros.actions import ActionClient
import sys
from enum import Enum
from switching_system.msg import (
    CmdPoseAction, CmdPoseFeedback, CmdPoseResult, CmdPoseGoal
)
import sensor_msgs as smsg
import geometry_msgs as gmsg
import control_msgs as cmsg
import queue
import rospy


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


class Status(Enum):
    ACTIVE = 1
    INACTIVE = 2


class State(object):
    """
    Class that holds state information for a particular state
    """

    def __init__(
            self,
            discrete_loc=PlannerState.INITIALISE,
            status=Status.ACTIVE):
        self._loc = discrete_loc

    @property
    def location(self):
        return self._loc


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


class AtTarget(Behaviour):
    """
    Implements a condition node to check if guard condition has been satisfied
    For now, it only checks cartesian guard conditions, if the distance between
    target and robot eef is within an epsilon ball
    TODO: Add callback option to check guard condition
    """

    def __init__(self, name="AtTarget"):
        """
        """
        super(AtTarget, self).__init__(name)
        self.blackboard = Blackboard()
        self.blackboard.goal_reached = False

    def initialise(self):
        self.blackboard.goal_reached = False
        pass

    def setup(self, timout=5):
        self.blackboard.goal_reached = False
        return True

    def update(self):
        new_status = py_trees.Status.FAILURE
        goal_reached = self.blackboard.goal_reached
        if goal_reached:
            new_status = Status.SUCCESS
        return new_status


class GetNextPose(Behaviour):
    """
    This class keeps track of a directed graph representing the state machine
    """

    def __init__(self, pose_fence, graph=None, name="GetNextPose"):
        super(GetNextPose, self).__init__(name)
        self._graph = graph
        self._pose_fence = pose_fence
        self._waypoints = None
        self._modes = None
        self.blackboard = Blackboard()

    def initialise(self):
        pass

    def setup(self, timeout=5):
        """
        Create the directed graph here. This grpah represents the evolution of
        the state machine
        """
        self._waypoints, self._modes = createDebugStateMachine(
            self._pose_fence)
        self.blackboard.X_Gcommand = self._waypoints.get()
        self.blackboard.Q = self._modes.get()
        return True

    def update(self):
        new_status = py_trees.Status.SUCCESS

        if self._waypoints.empty():
            new_status = py_trees.Status.FAILURE
        else:
            self.blackboard.X_Gcommand = self._waypoints.get()
            self.blackboard.Q = self._modes.get()

        return new_status


class MoveToPose(ActionClient):
    """
    Action leaf node to be consumed by a Behaviour Tree
    """

    def __init__(self,
                 name="Move to Pose",
                 action_namespace="cmd_pose",
                 action_spec=None,
                 action_goal=None,
                 override_feedback_message_on_running="moving"):
        super(MoveToPose, self).__init__(
            name=name,
            action_namespace=action_namespace,
            action_spec=action_spec,
            action_goal=action_goal,
            override_feedback_message_on_running=override_feedback_message_on_running)
        self.blackboard = Blackboard()
        # self.blackboard.X_Gcommand = gmsg.Pose()
        self.blackboard.is_at_target = False
        # goal_on_bb = self.blackboard.X_Gcommand
        # rospy.loginfo("setting up move to pose: {}".format(goal_on_bb))
        # self.action_goal = goal_on_bb

    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        rospy.loginfo("move to pose intialise")
        self.sent_goal = False
        goal_on_bb = self.blackboard.X_Gcommand
        self.action_goal = goal_on_bb

    def setup(self, timeout):
        super(MoveToPose, self).setup(timeout)
        return True


def constructBT(pose_fence):
    """
    Assembles an insertion state machine, instantiated as a BT via py_trees.
    The state machine is implemented as follows
    - repeat_dec
        - insert_seq
            - check_and_move_sel
                - at_target
                - move_to_pose
            - get_next_pose
    """
    # conditions nodes as guards
    at_target = AtTarget(name="at_target")  # guard condition

    # actions
    get_pose = GetNextPose(
        name="get_next_pose",
        pose_fence=pose_fence)
    # move = MoveToPose(name="move_to_pose")
    move = MoveToPose(
        action_namespace="cmd_pose",
        action_spec=CmdPoseAction,
        action_goal=CmdPoseGoal(),
        override_feedback_message_on_running="moving"
    )

    # control nodes
    insert_seq = Sequence(name="insert_seq", memory=False)
    move_composite = Selector(name="check_and_move_sel", memory=False)

    # root = Timeout(
    #     name="execute_dec",
    #     child=insert_seq,
    #     num_success=-1)
    root = insert_seq

    move_composite.add_children(
        [at_target, move]
    )
    insert_seq.add_children(
        [move_composite, get_pose]
    )
    return root


if __name__ == "__main__":
    createDebugStateMachine()
