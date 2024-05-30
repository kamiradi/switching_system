import numpy as np
import networkx as nx
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status
from py_trees.composites import Selector, Sequence
# from py_trees.decorators import Repeat
import py_trees_ros
from py_trees_ros.subscribers import ToBlackboard
from py_trees_ros.actions import ActionClient
import sys
from enum import Enum
from switching_system.msg import (
    CmdPoseAction, CmdPoseFeedback, CmdPoseResult, CmdPoseGoal
)
import sensor_msgs as smsg
import geometry_msgs as gmsg
import control_msgs.msg as cmsg
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
    GRASPREL = 13  # releases grasp


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

    waypoints.put(pose_fence['X_Ginsert'])
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

# BT Leaf nodes


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


class GraspCheck(Behaviour):
    """
    Implements a condition node to check if guard condition has been satisfied
    For now, it only checks cartesian guard conditions, if the distance between
    target and robot eef is within an epsilon ball
    TODO: Add callback option to check guard condition
    """

    def __init__(self, name="GraspCheck"):
        """
        """
        super(GraspCheck, self).__init__(name)
        self.blackboard = Blackboard()
        self.blackboard.grasp_action = False

    def initialise(self):
        pass

    def setup(self, timout=5):
        self.blackboard.grasp_action = False
        return True

    def update(self):
        """
        Should send Status FAILURE iff objects needs to be grasped
        """
        new_status = py_trees.Status.SUCCESS
        grasp_action = self.blackboard.grasp_action
        if grasp_action:
            new_status = py_trees.Status.FAILURE
        return new_status


class GetNextPose(Behaviour):
    """
    This class keeps track of a directed graph representing the state machine
    """

    def __init__(
            self,
            pose_fence,
            sm_func,
            graph=None,
            name="GetNextPose"):
        """
        This leaf node constructs the state machine and keeps track of the
        current state. It recieves a pose fence and a function that constructs
        a state machine.
        """
        super(GetNextPose, self).__init__(name)
        self._graph = graph
        self.sm_func = sm_func
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
        self._waypoints, self._modes = self.sm_func(
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

        mode = self.blackboard.Q
        # update grasp variable on blackboard
        if mode == PlannerState.GRASP:
            self.blackboard.grasp_action = True
        elif mode == PlannerState.GRASPREL:
            self.blackboard.grasp_action = True
        else:
            self.blackboard.grasp_action = False
        return new_status


class MoveToPose(ActionClient):
    """
    Action leaf node to be consumed by a Behaviour Tree. This node is
    responsible for moveing the robot to a given pose.
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


class Grasp(ActionClient):
    """
    Aciton leaf node responsible for executing the grasp action client.
    """

    def __init__(self,
                 name="Grasp",
                 action_namespace="",
                 action_spec=None,
                 action_goal=None,
                 override_feedback_message_on_running="grasping"):
        super(Grasp, self).__init__(
            name=name,
            action_namespace=action_namespace,
            action_spec=action_spec,
            action_goal=action_goal,
            override_feedback_message_on_running=override_feedback_message_on_running
        )
        self.blackboard = Blackboard()

    def initialise(self):
        self.sent_goal = False
        object_grasped = self.blackboard.object_grasped
        if not object_grasped:
            self.action_goal = self.grasp_goal
            self.blackboard.object_grasped = True
        else:
            self.action_goal = self.grasp_release_goal
            self.blackboard.object_grasped = False
        pass

    def setup(self, timeout):
        super(Grasp, self).setup(timeout)
        self.blackboard.object_grasped = False

        # grasp command
        self.grasp_goal = cmsg.GripperCommandGoal()
        cmd = cmsg.GripperCommand()
        cmd.position = 0.6
        cmd.max_effort = 50
        self.grasp_goal.command = cmd

        # grasp release command
        self.grasp_release_goal = cmsg.GripperCommandGoal()
        rel_cmd = cmsg.GripperCommand()
        rel_cmd.position = 0.0
        rel_cmd.max_effort = 50
        self.grasp_release_goal.command = rel_cmd
        return True


class forceToBlackboard(ToBlackboard):
    """
    Subscribes to the F_ext topic and writes the latest value to the
    blackboard. It also maintains a moving average of the force experienced by
    the robot, and a naive estimation of contact
    """

    def __init__(
        self,
            name,
            topic_name="/force_state_controller/F_ext",
            threshold=5.0):
        super(forceToBlackboard, self).__init__(
            name=name,
            topic_name=topic_name,
            topic_type=gmsg.WrenchStamped,
            blackboard_variables={"F_ext": None},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER
        )
        self.blackboard = Blackboard()
        self.blackboard.F_ext = gmsg.WrenchStamped()
        # self.blackboard.F_ext_avg = 0.0
        self.contact_threshold = threshold

    def update(self):
        """
        Calls the parent to write the raw data to the blackboard. Additionally
        it also maintains a moving average of the force, and checks if contact
        threshold has been exceeded.
        """
        status = super(forceToBlackboard, self).update()
        if self.blackboard.F_ext.wrench.force.z > self.contact_threshold:
            self.blackboard.contact_estimated = True
        else:
            self.blackboard.contact_estimated = False
        return status

# BT Construction


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
    # blackboard update nodes
    # force_external =
    # conditions nodes as guards
    at_target = AtTarget(name="at_target")  # guard condition

    # actions
    get_pose = GetNextPose(
        name="get_next_pose",
        pose_fence=pose_fence,
        sm_func=createDebugStateMachine
    )
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


def GraspBT(pose_fence):
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
        pose_fence=pose_fence,
        sm_func=createGraspStateMachine
    )
    # move = MoveToPose(name="move_to_pose")
    move = MoveToPose(
        action_namespace="cmd_pose",
        action_spec=CmdPoseAction,
        action_goal=CmdPoseGoal(),
        override_feedback_message_on_running="moving"
    )

    grasp = Grasp(
        action_namespace="gripper_controller/gripper_cmd",
        action_spec=cmsg.GripperCommandAction,
        action_goal=cmsg.GripperCommandGoal(),
        override_feedback_message_on_running="grasping"
    )

    # control nodes
    insert_seq = Sequence(name="insert_seq", memory=False)
    move_composite = Selector(name="check_and_move_sel", memory=False)
    grasp_composite = Selector(name="check_and_grasp_sel", memory=False)
    should_grasp = GraspCheck(name="grasp_check")

    # root = Timeout(
    #     name="execute_dec",
    #     child=insert_seq,
    #     num_success=-1)
    root = insert_seq

    grasp_composite.add_children(
        [should_grasp, grasp]
    )
    move_composite.add_children(
        [at_target, move]
    )
    insert_seq.add_children(
        [move_composite, grasp_composite, get_pose]
    )
    return root


def EstimationBT(pose_fence):
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
    # blackboard update nodes
    force_updateBB = forceToBlackboard(name="F_extBB")

    # conditions nodes as guards
    at_target = AtTarget(name="at_target")  # guard condition

    # actions
    get_pose = GetNextPose(
        name="get_next_pose",
        pose_fence=pose_fence,
        sm_func=createEstimationStateMachine
    )
    # move = MoveToPose(name="move_to_pose")
    move = MoveToPose(
        action_namespace="cmd_pose",
        action_spec=CmdPoseAction,
        action_goal=CmdPoseGoal(),
        override_feedback_message_on_running="moving"
    )

    grasp = Grasp(
        action_namespace="gripper_controller/gripper_cmd",
        action_spec=cmsg.GripperCommandAction,
        action_goal=cmsg.GripperCommandGoal(),
        override_feedback_message_on_running="grasping"
    )

    # control nodes
    insert_composite_seq = Sequence(name="insert_seq", memory=False)
    move_composite_sel = Selector(name="check_and_move_sel", memory=False)
    grasp_composite_sel = Selector(name="check_and_grasp_sel", memory=False)
    should_grasp = GraspCheck(name="grasp_check")

    # root = Timeout(
    #     name="execute_dec",
    #     child=insert_seq,
    #     num_success=-1)
    root = insert_composite_seq

    grasp_composite_sel.add_children(
        [should_grasp,
         grasp]
    )
    move_composite_sel.add_children(
        [at_target,
         move]
    )
    insert_composite_seq.add_children(
        [force_updateBB,
         move_composite_sel,
         grasp_composite_sel,
         get_pose]
    )
    return root


def noTargetCheckBT(pose_fence):
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
    # at_target = AtTarget(name="at_target")  # guard condition

    # actions
    get_pose = GetNextPose(
        name="get_next_pose",
        pose_fence=pose_fence,
        sm_func=createDebugStateMachine
    )
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

    # move_composite.add_children(
    #     [at_target, move]
    # )
    insert_seq.add_children(
        [move, get_pose]
    )
    return root


if __name__ == "__main__":
    createDebugStateMachine()
