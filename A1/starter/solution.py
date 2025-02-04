#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

PROBLEMS[20].print_state()

# SOKOBAN HEURISTICS
def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.

    # HEURISTIC THAT CALCULATES THE AMOUNT OF OBSTACLES IN THE MIDDLE OF THE BOX AND STORAGE
    min_distance = math.inf
    total_obstacles = 0
    selected_storage = {}
    for box in state.boxes:
        for storage in state.storage:
            if box == storage:
                break
            if box != storage:
                distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
                min_distance = min(min_distance, distance)
                if min_distance == distance:
                    selected_storage[box] = storage
        min_distance = math.inf
    
    for box, storage in selected_storage[box].items():
        obstacles_in_path = 0
        

        x_min, x_max = min(box[0], storage[0]), max(box[0], storage[0])
        y_min, y_max = min(box[1], storage[1]), max(box[1], storage[1])

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                if (x, y) in state.obstacles:
                    obstacles_in_path += 1
        
        total_obstacles += obstacles_in_path

    return total_obstacles

print(heur_alternate(PROBLEMS[20]))

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    manhattan = 0 #initialize manhattan distance
    man_options = [] #initialize list of manhattan distances
    min_distance = math.inf
    for box in state.boxes:
        for storage in state.storage:
            if box == storage:
                break
            if box != storage:
                distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
                min_distance = min(min_distance, distance)
                #print(min_distance)
        manhattan += min_distance
        min_distance = math.inf
                
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    return manhattan  # CHANGE THIS

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return 0 #CHANGE THIS

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT    
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''

    open_list = [initial_state]
    closed_list = set()

    start_node = sNode(initial_state, 0, heur_fn(initial_state))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        current_state = current_node.state

        if current_state.is_goal():
            return current_state, SearchStats(time.time() - start_time, len(closed_list), len(open_list))

        closed_list.add(current_state)

        for neighbor in current_state.successors():
            if neighbor not in closed_list:
                neighbor_node = sNode(neighbor, current_node.gval + 1, heur_fn(neighbor))
                heapq.heappush(open_list, neighbor_node)

    return None, None  # CHANGE THIS

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    return None, None #CHANGE THIS

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    return None, None #CHANGE THIS



