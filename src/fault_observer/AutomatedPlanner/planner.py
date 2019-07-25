from __future__ import print_function
from time import time
import heapq

def planner(problem, heuristic=None, state0=None, goal=None,
            monotone=False, verbose=True):
    """
    Implements A* search to find a plan for the given problem.
    Arguments:
    problem   - a pyddl Problem
    heuristic - a heuristic to use (h(state) = 0 by default)
    state0    - initial state (problem.initial_state by default)
    goal      - tuple containing goal predicates and numerical conditions
                (default is (problem.goals, problem.num_goals))
    monotone  - if True, only applies actions by ignoring delete lists
    verbose   - if True, prints statistics before returning
    """
    if heuristic is None:
        heuristic = null_heuristic
    if state0 is None:
        state0 = problem.problem.initial_state
    if goal is None:
        goal = problem.problem.goals


    states_explored = 0
    closed = set()
    fringe = [(heuristic(state0), -state0.cost, state0)]
    heapq.heapify(fringe)
    start = time()
    while True:
        if len(fringe) == 0:
            if verbose: print('States Explored: %d' % states_explored)
            return None

        # Get node with minimum evaluation function from heap
        h, _, node = heapq.heappop(fringe)
        states_explored += 1

        # Goal test
        if node.is_true(goal):
            plan = node.plan()
            dur = time() - start
            # Uncomment this to print all the properties of the plans

            # if verbose:
            #     print('States Explored: %d' % states_explored)
            #     print('Time per state: %.3f ms' % (1000*dur / states_explored))
            #     print('Plan length: %d' % node.cost)

            return plan

        # Expand node if we haven't seen it before
        if node not in closed:
            closed.add(node)

            # Apply all applicable actions to get successors
            successors = []
            for action in problem.problem.grounded_actions:
                if node.is_true(action.preconditions):
                    successors.append(node.apply(action, monotone))
            successors = set(successors)

            # Compute heuristic and add to fringe
            for successor in successors:
                if successor not in closed:
                    f = successor.cost + heuristic(successor)
                    heapq.heappush(fringe, (f, -successor.cost, successor))


########## HEURISTICS ##########

def null_heuristic(state):
    """Admissible, but trivial heuristic"""
    return 0

def plan_cost(plan):
    """Convert a plan to a cost, handling nonexistent plans"""
    if plan is None:
        return float('inf')
    else:
        return len(plan)
