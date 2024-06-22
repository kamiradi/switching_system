import numpy as np
import networkx as nx
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status
from py_trees.composites import Selector, Sequence, Parallel
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
import geometry_msgs.msg as gmsg
import control_msgs.msg as cmsg
import queue
import rospy
from states import *
from task_compilation import *

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


class ContactCheck(Behaviour):
    """
    Implements a condition node to check if failure guard condition has been
    satisfied. It reads the `contact_estimated` variable from the blackboard.
    To be incorporated in a selector composite (this check needs to be given
    highest priority)
    """

    def __init__(self, name="ContactCheck"):
        super(ContactCheck, self).__init__(name)
        self.blackboard = Blackboard()
        self.blackboard.contact_estimated = False

    def initialise(self):
        pass

    def setup(self):
        self.blackboard.contact_estimated = False
        return True

    def update(self):
        """
        Should send status FAILURE iff contact estimated
        """
        new_status = py_trees.Status.SUCCESS
        contact = self.blackboard.contact_estimated
        if contact:
            new_status = py_trees.Status.FAILURE
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

    def setup(self, timeout=5):
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
    This class keeps track of a queue representing the state machine
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
        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)

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
            self.blackboard.Q = self._modes.get()
            rospy.loginfo("Mode: {}".format(self.blackboard.Q))

        mode = self.blackboard.Q
        # update grasp variable on blackboard
        if mode == PlannerState.GRASP:
            self.blackboard.grasp_action = True
            self.blackboard.X_Gcommand = self._waypoints.get()
        elif mode == PlannerState.GRASPREL:
            self.blackboard.grasp_action = True
            self.blackboard.X_Gcommand = self.cmdpose_goal
            self._waypoints.get()
        elif mode == PlannerState.INSERTRES:
            rospy.loginfo("inser result command execution")
            tf_X_Eeff = self._tfBuffer.lookup_transform(
                'panda_link0',
                'result',
                rospy.Time())
            translation = tf_X_Eeff.transform.translation
            orientation = tf_X_Eeff.transform.rotation
            goal_pose = gmsg.Pose()
            goal_pose.position.x = translation.x
            goal_pose.position.y = translation.y
            goal_pose.position.z = translation.z
            goal_pose.orientation.x = orientation.x
            goal_pose.orientation.y = orientation.y
            goal_pose.orientation.z = orientation.z
            goal_pose.orientation.w = orientation.w
            cmdpose_goal = CmdPoseGoal()
            cmdpose_goal.pose.pose = goal_pose
            cmdpose_goal.pose.use_estimation = False

            self.cmdpose_goal = cmdpose_goal
            self.blackboard.X_Gcommand = cmdpose_goal
            self._waypoints.get()
        else:
            self.blackboard.grasp_action = False
            self.blackboard.X_Gcommand = self._waypoints.get()
        return new_status


class GetNextPoseCompiled(Behaviour):
    """
    This class keeps track of a queue representing the state machine. It
    recieves a compiled task, represented as a dictionary of waypoints and
    modes.
    """

    def __init__(
            self,
            compiled_task,
            graph=None,
            name="GetNextPose"):
        """
        This leaf node constructs the state machine and keeps track of the
        current state. It recieves a pose fence and a function that constructs
        a state machine.
        """
        super(GetNextPoseCompiled, self).__init__(name)
        self._graph = graph
        self._waypoints = compiled_task['waypoints']
        self._modes = compiled_task['modes']
        self.blackboard = Blackboard()
        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)

    def initialise(self):
        pass

    def setup(self, timeout=5):
        """
        Create the directed graph here. This grpah represents the evolution of
        the state machine
        """
        self.blackboard.X_Gcommand = self._waypoints.get()
        self.blackboard.Q = self._modes.get()
        return True

    def update(self):
        new_status = py_trees.Status.SUCCESS

        if self._waypoints.empty():
            new_status = py_trees.Status.FAILURE
        else:
            self.blackboard.Q = self._modes.get()

        mode = self.blackboard.Q
        # update grasp variable on blackboard
        if mode == PlannerState.GRASP:
            self.blackboard.X_Gcommand = self._waypoints.get()
            self.blackboard.grasp_action = True
        elif mode == PlannerState.GRASPREL:
            self._waypoints.get()
            self.blackboard.X_Gcommand = self.cmdpose_goal
            self.blackboard.grasp_action = True
        elif mode == PlannerState.INSERTRES:
            rospy.loginfo("inser result command execution")
            tf_X_Eeff = self._tfBuffer.lookup_transform(
                'panda_link0',
                'result',
                rospy.Time())
            translation = tf_X_Eeff.transform.translation
            orientation = tf_X_Eeff.transform.rotation
            goal_pose = gmsg.Pose()
            goal_pose.position.x = translation.x
            goal_pose.position.y = translation.y
            goal_pose.position.z = translation.z
            goal_pose.orientation.x = orientation.x
            goal_pose.orientation.y = orientation.y
            goal_pose.orientation.z = orientation.z
            goal_pose.orientation.w = orientation.w
            cmdpose_goal = CmdPoseGoal()
            cmdpose_goal.pose.pose = goal_pose
            cmdpose_goal.pose.use_estimation = False

            self.cmdpose_goal = cmdpose_goal
            self.blackboard.X_Gcommand = cmdpose_goal
            self._waypoints.get()
        else:
            self.blackboard.grasp_action = False
            self.blackboard.X_Gcommand = self._waypoints.get()
        return new_status


class GetNextState(Behaviour):
    """
    This class modifies the above `GetNextPose` to implement a directed graph
    to maintain the state machine as opposed to a queue. Remaining
    functionality is the same
    """

    def __init__(
        self,
        pose_fence,
        sm_func,
        graph=None,
            name="GetNextState"):
        super(GetNextState, self).__init__(name)
        self._graph = graph
        self.sm_func = sm_func
        self._pose_fence = pose_fence
        self.blackboard = Blackboard()
        self.state = None

    def intialise(self):
        pass

    def setup(self, timeout=5):
        self.state_machine = self.sm_func(
            self._pose_fence
        )
        self.state = self.state_machine.reset()
        self.blackboard.X_Gcommand = self.state.goal
        self.blackboard.Q = self.state.planning_state

    def update(self):
        new_status = py_trees.Status.SUCCESS
        # HERE


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


class ForceToBlackboard(ToBlackboard):
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
        super(ForceToBlackboard, self).__init__(
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
        status = super(ForceToBlackboard, self).update()
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
    # force_updateBB = ForceToBlackboard(name="F_extBB")

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
        [move_composite_sel,
         grasp_composite_sel,
         get_pose]
    )
    return root


def BT(compiled_task):
    """
    Assembles an insertion state machine, instantiated as a BT via py_trees.
    The state machine is implemented as follows
    - repeat_dec
        - insert_seq
            - check_and_move_sel
                - at_target
                - move_to_pose
            - get_next_pose

    Accepts a dictionary of the waypoints and modes of the compiled task
    """
    # blackboard update nodes
    force_updateBB = ForceToBlackboard(name="F_extBB")

    topics_BB_seq = Sequence(name="topics_seq", memory=False)

    # conditions nodes as guards
    at_target = AtTarget(name="at_target")  # guard condition

    # actions
    get_pose = GetNextPoseCompiled(
        name="get_next_pose",
        compiled_task=compiled_task
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
    root_par = Parallel(name="state_machine_par")
    insert_composite_seq = Sequence(name="insert_seq", memory=False)
    move_composite_sel = Selector(name="check_and_move_sel", memory=False)
    grasp_composite_sel = Selector(name="check_and_grasp_sel", memory=False)
    should_grasp = GraspCheck(name="grasp_check")

    # root = Timeout(
    #     name="execute_dec",
    #     child=insert_seq,
    #     num_success=-1)
    root = root_par

    root_par.add_children(
        [topics_BB_seq,
         insert_composite_seq]
    )

    topics_BB_seq.add_children(
        [force_updateBB]
    )

    grasp_composite_sel.add_children(
        [should_grasp,
         grasp]
    )
    move_composite_sel.add_children(
        [at_target,
         move]
    )
    insert_composite_seq.add_children(
        [move_composite_sel,
         grasp_composite_sel,
         get_pose]
    )
    return root


def LissajousBT(pose_fence):
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
    # force_updateBB = ForceToBlackboard(name="F_extBB")

    # conditions nodes as guards
    at_target = AtTarget(name="at_target")  # guard condition

    # actions
    get_pose = GetNextPose(
        name="get_next_pose",
        pose_fence=pose_fence,
        sm_func=createLissajousStateMachine
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
        [move_composite_sel,
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
