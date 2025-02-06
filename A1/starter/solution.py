#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

# SOKOBAN HEURISTICS
def is_deadlocked(box, state):
    """Check if a box is in a deadlock position."""
    x, y = box
    walls = state.obstacles

    if box in state.storage:
        return False

    if ((x - 1, y) in walls and (x, y - 1) in walls) or \
       ((x - 1, y) in walls and (x, y + 1) in walls) or \
       ((x + 1, y) in walls and (x, y - 1) in walls) or \
       ((x + 1, y) in walls and (x, y + 1) in walls):
        return True

    if ((x - 1, y) in walls and (x + 1, y) in walls) or \
       ((x, y - 1) in walls and (x, y + 1) in walls):

        row_storages = [s for s in state.storage if s[1] == y]
        col_storages = [s for s in state.storage if s[0] == x]

        if ((x - 1, y) in walls and (x + 1, y) in walls and box not in row_storages) or \
           ((x, y - 1) in walls and (x, y + 1) in walls and box not in col_storages):
            return True  

    enclosed_boxes, enclosed_storages = find_enclosed_boxes_and_storage(box, state)

    if len(enclosed_boxes) > len(enclosed_storages):  
        return True  

    return False


def find_enclosed_boxes_and_storage(start_box, state):
    """Find all boxes and storage spots within a confined space where movement is blocked."""
    queue = [start_box]
    visited = set()
    enclosed_boxes = set()
    enclosed_storages = set()

    walls = state.obstacles  
    boundary = set()  

    while queue:
        box = queue.pop()
        if box in visited:
            continue
        visited.add(box)

        if box in state.boxes:
            enclosed_boxes.add(box)
        if box in state.storage:
            enclosed_storages.add(box)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (box[0] + dx, box[1] + dy)

            if neighbor in visited:
                continue

            if neighbor in walls:  
                continue

            if neighbor not in state.boxes:  
                boundary.add(neighbor)
                continue

            queue.append(neighbor)

    if boundary:
        return set(), set()

    return enclosed_boxes, enclosed_storages

def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.
    for box in state.boxes:
        if is_deadlocked(box, state):
            return math.inf  

    unpaired_boxes = list(state.boxes)
    unpaired_storage = list(state.storage)
    
    box_storage_pairs = {}

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

    total_cost = 0

    for box, storage in box_storage_pairs.items():
        distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
        obstacles_in_path = 0

        x_min, x_max = min(box[0], storage[0]), max(box[0], storage[0])
        y_min, y_max = min(box[1], storage[1]), max(box[1], storage[1])

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                if (x, y) in state.obstacles:
                    obstacles_in_path += 1

        total_cost += distance + 2 * obstacles_in_path #16/25

    return total_cost

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    manhattan = 0  # Initialize total Manhattan distance
    
    for box in state.boxes:
        # Find the closest storage location for each box
        min_distance = min(abs(box[0] - storage[0]) + abs(box[1] - storage[1]) for storage in state.storage)
        manhattan += min_distance
    
    return manhattan

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    what_to_return = sN.gval + weight * sN.hval
    return what_to_return

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    engine = SearchEngine(strategy="custom")
    engine.init_search(
        initState=initial_state,
        goal_fn=sokoban_goal_state,
        heur_fn=heur_fn,
        fval_function=(lambda node: fval_function(node, weight))
    )
    sta, statas = engine.search(timebound=timebound)
    return (sta, statas)

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    start_time = os.times()[0]  # Record the start time
    best_solution = None
    best_cost = float('inf')
    
    # Initialize the search engine
    se = SearchEngine('custom')
    
    while weight >= 1:
        # Calculate remaining time
        elapsed_time = os.times()[0] - start_time
        remaining_time = timebound - elapsed_time
        
        if remaining_time <= 0:
            # Time is up, return the best solution found so far
            return best_solution, stats if best_solution else (False, stats)
        
        # Define the f-value function for Weighted A* with the current weight
        def fval_function(sN):
            return sN.gval + weight * sN.hval
        
        # Initialize the search with the current weight
        se.init_search(initial_state, sokoban_goal_state, heur_fn, fval_function)
        
        # Perform Weighted A* search with the remaining time and cost bound
        state, stats = se.search(remaining_time, costbound=(float('inf'), float('inf'), best_cost))
        
        if state:
            # A solution was found
            if state.gval < best_cost:
                # Update the best solution and its cost
                best_solution = state
                best_cost = state.gval
        
        # Decrease the weight for the next iteration
        weight -= 0.1  # Adjust the decrement step as needed
        
    return best_solution, stats if best_solution else (False, stats)

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    start_time = os.times()[0]  # Record the start time
    best_solution = None
    best_cost = float('inf')
    
    # Initialize the search engine
    se = SearchEngine('custom')
    
    # Define a custom f-value function for GBFS (only uses h-value)
    def gbfs_fval_function(sN):
        return sN.hval
    
    se.init_search(initial_state, sokoban_goal_state, heur_fn, gbfs_fval_function)
    
    while True:
        # Calculate remaining time
        elapsed_time = os.times()[0] - start_time
        remaining_time = timebound - elapsed_time
        
        if remaining_time <= 0:
            # Time is up, return the best solution found so far
            return best_solution, stats if best_solution else (False, stats)
        
        # Perform GBFS search with the remaining time
        state, stats = se.search(remaining_time, costbound=(float('inf'), float('inf'), best_cost))
        
        if state:
            # A solution was found
            if state.gval < best_cost:
                # Update the best solution and its cost
                best_solution = state
                best_cost = state.gval
        else:
            # No more solutions can be found within the time bound
            return best_solution, stats if best_solution else (False, stats)



