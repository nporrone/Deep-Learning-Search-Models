# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    from util import Stack

    fringe = Stack()  # store nodes in the fringe (as a stack) and there paths
    visited = set()  # Create a set which holds all visited nodes
    fringe.push((problem.getStartState(), []))  # Store the start node and its path

    while not fringe.isEmpty():  # while that stack is not empty execute
        node, path = fringe.pop()  # Pop the element you wish to step into from the stack and store its node/path
        if problem.isGoalState(node):  # check if the node is the goal state
            break  # Exit the loop, you have found the goal state
        else:
            if node not in visited:  # Stack may contain the same node twice so we save run time by skipping
                visited.add(node)  # add the node to the visted set
                children = problem.getSuccessors(node)  # store the children of the node
                for child in children:
                    fringe.push((child[0], path + [child[1]]))  # push child into the fringe with its correct path

    return path  # list of actions to reach the goal


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    from util import Queue

    fringe = Queue()  # store nodes in the fringe(as a queue)
    visted = set()  # create a set to hold all the visted nodes
    fringe.push((problem.getStartState(), []))  # Push the starting node and its path to the fringe

    while not fringe.isEmpty():
        
        node, path = fringe.pop()  # pop the current element from the stack
        if problem.isGoalState(node):  # check if current node is the goal node
            break  # if it is, break the loop and return the nodes path
        else:  # if not, check its children
            if node not in visted:  # check to see if you have visited this node before
                visted.add(node)  # if you havent, mark the node as visited
                children = problem.getSuccessors(node)  # store the children of the node
                for child in children:
                    fringe.push((child[0], path + [child[1]]))

    return path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    from util import PriorityQueue

    fringe = PriorityQueue()  # store nodes in the fringe(as a priority queue)
    visted = set()  # create a set to hold all the visted nodes
    fringe.push((problem.getStartState(), [], 0), 0)  # Push the starting node and its path to the fringe
                                                      # along with its cost and its path cost
    while not fringe.isEmpty():
        node, path, cost = fringe.pop()  # pop the current element from the stack and store its node, path and cost
        if problem.isGoalState(node):  # check if current node is the goal node
            break  # if it is, break the loop and return the nodes path
        else:  # if not, check its children
            if node not in visted:  # check to see if you have visited this node before
                visted.add(node)  # if you havent, mark the node as visited
                children = problem.getSuccessors(node)  # store the children of the node
                for child in children:
                    fringe.push((child[0], path + [child[1]], child[2]), cost + child[2])  # push the child with its path
                                                                                      # cost

    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    from util import PriorityQueue

    fringe = PriorityQueue()  # store nodes in the fringe(as a priority queue)
    visted = set()  # create a set to hold all the visted nodes
    fringe.push((problem.getStartState(), [], 0, 0), heuristic(problem.getStartState(), problem) + 0)  # Push the starting node and its path to the fringe
                                                      # along with its cost and its path cost (w/ heursitics)
    while not fringe.isEmpty():
        current_element = fringe.pop()  # pop the current element from the stack
        node = current_element[0]
        path = current_element[1]
        cost = current_element[2]
        if problem.isGoalState(node):  # check if current node is the goal node
            break  # if it is, break the loop and return the nodes path
        else:  # if not, check its children
            if node not in visted:  # check to see if you have visited this node before
                visted.add(node)  # if you havent, mark the node as visited
                children = problem.getSuccessors(node)  # store the children of the node
                for child in children:
                    fringe.push((child[0], path + [child[1]], child[2]), heuristic(child[0], problem) + cost)  # push the child with its path
                                                                                                   # cost and path cost
    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
