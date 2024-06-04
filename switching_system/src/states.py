import networkx as nx
from enum import Enum


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


class TransitionInput(Enum):
    GOAL = 1
    FAILURE = 2


class State(object):
    def __init__(self, planner_state):
        self.planner_state = planner_state
        pass

    def __call__(self, input):
        raise NotImplementedError

    def connect(self, next_state, failure_state):
        """
        Takes in a networkx graph G, adds an edge between this state and the
        next state, this state and the failure state
        """
        self.next_state = next_state
        self.failure_state = failure_state


class InitState(State):
    def __init__(self, goal, planner_state=PlannerState.INITIALISE):
        super(InitState, self).__init__(planner_state)
        self.goal = goal

    def __call__(self, input):
        if input == TransitionState.GOAL:
            return self.next_state
        elif input == TransitionState.FAILURE:
            return self.failure_state


class GraspproachState(State):
    def __init__(self, goal, planner_state=PlannerState.GRASPPROACH):
        super(GraspproachState, self).__init__(planner_state)
        self.goal = goal


class ApproachState(State):
    def __init__(self, goal, planner_state=PlannerState.APPROACH):
        super(ApproachState, self).__init__(planner_state)
        self.goal = goal


class FailureState(State):
    def __init__(self, goal, planner_state=PlannerState.FAILURE):
        super(FailureState, self).__init__(planner_state)
        self.goal = goal


class StateMachine():
    def __init__(self, pose_fence):
        self.init_state = self.build()
        self._pose_fence = pose_fence

    def build(self):
        raise NotImplementedError

    def reset(self):
        self.state = self.init_state

    def __call__(self, input):
        prev_state = self.state
        self.state(input)


class DebugStateMachine(StateMachine):
    def build(self):
        self.intialise = InitState(goal=self._pose_fence['X_Gstart'])
        self.graspproach = GraspproachState(
            goal=self._pose_fence['X_Pregrasp'])
        self.approach = ApproachState(goal=self._pose_fence['X_Preinsert'])
        self.failure = FailureState(goal=self._pose_fence['X_Gstart'])

        self.intialise.connect(
            next_state=self.graspproach,
            failure_state=self.failure
        )
        self.graspproach.connect(
            next_state=self.approach,
            failure_state=self.failure
        )
        return self.initialise
        # self.approach.connect(
        #    next_state=self.initialise,
        #    failure_state=self.failure
        # )


if __name__ == "__main__":
    state_machine = DebugStateMachine()
    print(list(state_machine.graph.nodes))
    pass
