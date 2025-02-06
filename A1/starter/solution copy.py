#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
import time
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

#PROBLEMS[5].print_state()

# SOKOBAN HEURISTICS
""" obstacles 7/25
def heur_alternate(state): #obstacles
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
    
    for box, storage in selected_storage.items():
        obstacles_in_path = 0

        x_min, x_max = min(box[0], storage[0]), max(box[0], storage[0])
        y_min, y_max = min(box[1], storage[1]), max(box[1], storage[1])

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                if (x, y) in state.obstacles:
                    obstacles_in_path += 1
        
        total_obstacles += obstacles_in_path

    return total_obstacles
"""
""" greedy pairing 19/25
def heur_alternate(state): #greedy pairing
    '''Improved heuristic using greedy pairing of boxes to storage and obstacle penalties.'''
    
    # List of unpaired boxes and storage locations
    unpaired_boxes = list(state.boxes)
    unpaired_storage = list(state.storage)
    
    # Pairing dictionary: box -> assigned storage
    box_storage_pairs = {}

    # Greedy pairing: assign each box to the closest available storage
    while unpaired_boxes and unpaired_storage:
        best_pair = None
        min_distance = math.inf

        for box in unpaired_boxes:
            for storage in unpaired_storage:
                distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
                if distance < min_distance:
                    min_distance = distance
                    best_pair = (box, storage)

        if best_pair:
            box, storage = best_pair
            box_storage_pairs[box] = storage
            unpaired_boxes.remove(box)
            unpaired_storage.remove(storage)

    # Compute total cost based on Manhattan distance + obstacles in path
    total_cost = 0

    for box, storage in box_storage_pairs.items():
        distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
        obstacles_in_path = 0

        x_min, x_max = min(box[0], storage[0]), max(box[0], storage[0])
        y_min, y_max = min(box[1], storage[1]), max(box[1], storage[1])

        # Count obstacles along the Manhattan path
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                if (x, y) in state.obstacles:
                    obstacles_in_path += 1

        # Combine distance with obstacle penalty
        total_cost += distance + 2 * obstacles_in_path  # Penalize obstacles more heavily

    return total_cost
"""

""" deadlock logic try 1
def is_deadlocked(box, state):
    x, y = box
    walls = state.obstacles.union(state.boxes)  # Walls and other boxes act as obstacles

    if box in state.storage:
        return False  # Already in storage, not stuck

    # Corner deadlock (box against two walls)
    if ((x - 1, y) in walls and (x, y - 1) in walls) or \
       ((x - 1, y) in walls and (x, y + 1) in walls) or \
       ((x + 1, y) in walls and (x, y - 1) in walls) or \
       ((x + 1, y) in walls and (x, y + 1) in walls):
        return True

    # Wall deadlock (box trapped between two walls)
    if (x - 1, y) in walls and (x + 1, y) in walls:
        return True
    if (x, y - 1) in walls and (x, y + 1) in walls:
        return True

    return False
"""

def is_deadlocked(box, state):
    """Check if a box is in a deadlock position."""
    x, y = box
    walls = state.obstacles

    if box in state.storage:
        return False  # If already in storage, it's fine

    # Corner deadlock: box is stuck in a corner
    if ((x - 1, y) in walls and (x, y - 1) in walls) or \
       ((x - 1, y) in walls and (x, y + 1) in walls) or \
       ((x + 1, y) in walls and (x, y - 1) in walls) or \
       ((x + 1, y) in walls and (x, y + 1) in walls):
        return True

    # Wall deadlock: box is stuck between two walls
    if ((x - 1, y) in walls and (x + 1, y) in walls) or \
       ((x, y - 1) in walls and (x, y + 1) in walls):
        return True

    return False


def find_enclosed_boxes_and_storage(start_box, state):
    """Find all boxes and storage spots within a confined space where movement is blocked."""
    queue = [start_box]
    visited = set()
    enclosed_boxes = set()
    enclosed_storages = set()

    walls = state.obstacles  # Only consider obstacles as walls
    boundary = set()  # Tracks if we touch an open space

    while queue:
        box = queue.pop()
        if box in visited:
            continue
        visited.add(box)

        if box in state.boxes:
            enclosed_boxes.add(box)
        if box in state.storage:
            enclosed_storages.add(box)

        # Check adjacent spaces
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (box[0] + dx, box[1] + dy)

            if neighbor in visited:
                continue

            if neighbor in walls:  # Walls block expansion
                continue

            if neighbor not in state.boxes:  # If we reach open space, we are NOT enclosed
                boundary.add(neighbor)
                continue

            queue.append(neighbor)

    # If we touch open space, it's not an enclosed region
    if boundary:
        return set(), set()

    return enclosed_boxes, enclosed_storages



def heur_alternate(state):
    """Improved heuristic using greedy assignment and Manhattan distance."""
    boxes = list(state.boxes)
    storage = list(state.storage)
    total_distance = 0

    # Greedy assignment: assign each box to the closest available storage
    while boxes and storage:
        min_distance = math.inf
        best_box, best_storage = None, None

        for box in boxes:
            for st in storage:
                distance = abs(box[0] - st[0]) + abs(box[1] - st[1])
                if distance < min_distance:
                    min_distance = distance
                    best_box, best_storage = box, st

        if best_box and best_storage:
            total_distance += min_distance
            boxes.remove(best_box)
            storage.remove(best_storage)

    return total_distance




print(heur_alternate(PROBLEMS[3]))

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

    # f(n) = g(n) + w * h(n)
    return sN.gval + weight * sN.hval

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT    
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''

    se = SearchEngine('custom')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    return se.search(timebound)

def iterative_astar(initial_state, heur_fn, weight=10, timebound=5):
    start = os.times()[0]
    ans, ansCost = None, float('inf')
    while timebound > os.times()[0] - start:
        search_engine = SearchEngine(strategy='astar', cc_level='full')
        wrapped_fval_function = (lambda sN: fval_function(sN, weight))
        search_engine.init_search(initState=initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn, fval_function=wrapped_fval_function)
        endState, stats = search_engine.search(timebound=max(0, timebound - (os.times()[0] - start) - 0.02), costbound=(ansCost, math.inf, math.inf))

        if endState and endState.gval < ansCost:
            ans = (endState, stats)
            ansCost = endState.gval

        weight *= 0.8  # Reduce weight gradually
    return ans

#iterative_astar(PROBLEMS[2], heur_alternate, 1, 5)


"""
best_solution = None
    best_cost = float('inf')  # Track cost of best solution found
    start_time = os.times()[0]

    while os.times()[0] - start_time < timebound:
        remaining_time = timebound - (os.times()[0] - start_time)
        if remaining_time <= 0:
            break  # Stop if no time remains

        # Run weighted A* search
        success, stats = weighted_astar(initial_state, heur_fn, weight, remaining_time)

        if success:
            cost = stats.gval  # Assume `stats` has a `gval` field for the solution cost
            if cost < best_cost:
                best_solution = success  # Store the solution (assuming success represents the solution)
                best_cost = cost  # Update best cost for pruning

        # Reduce weight for the next iteration (move toward optimal A*)
        weight = max(1, weight * 0.5)  # Avoid going below 1 (standard A*)

        if weight == 1:  # Standard A* reached, stop refining
            break

    return best_solution
"""
    
#iterative_astar(PROBLEMS[5], heur_manhattan_distance, 1, 5)

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''

    """
    start = os.times()[0]
    ans, ansCost = None, float('inf')

    while (timebound > os.times()[0] - start):
        search_engine = SearchEngine(strategy = 'best_first', cc_level = 'full')
        search_engine.init_search(initState = initial_state, goal_fn = sokoban_goal_state, heur_fn = heur_fn)
        endState, stats = search_engine.search(timebound=max(0, timebound - (os.times()[0] - start) - 0.02), costbound=(ansCost, ansCost, ansCost))

        if endState and endState.gval < ansCost:
                ans = (endState, stats)
                ansCost = endState.gval
    
    return ans
    """
    
    
    
    """
    best_solution = None
    best_cost = float('inf')  # Best path cost found
    start_time = os.times()[0]

    while os.times()[0] - start_time < timebound:
        # Run GBFS with a pruning condition
        remaining_time = timebound - (os.times()[0] - start_time)
        if remaining_time < 0:
            break
        
        se = SearchEngine('best_first', 'default')
        se.init_search(initial_state, sokoban_goal_state, heur_fn)
        solution, stats = se.search(remaining_time, costbound=None)

        if solution:
            cost = stats.gval
            if cost < best_cost:
                best_solution = solution
                best_cost = cost

    return best_solution, stats
    """

#for i in range(6):
 #   iterative_gbfs(PROBLEMS[i], heur_manhattan_distance, 5)


