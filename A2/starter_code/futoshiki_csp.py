# Look for #IMPLEMENT tags in this file.
'''
All models need to return a CSP object, and a list of lists of Variable objects
representing the board. The returned list of lists is used to access the
solution.

For example, after these three lines of code

    csp, var_array = futoshiki_csp_model_1(board)
    solver = BT(csp)
    solver.bt_search(prop_FC, var_ord)

var_array[0][0].get_assigned_value() should be the correct value in the top left
cell of the Futoshiki puzzle.

1. futoshiki_csp_model_1 (worth 20/100 marks)
    - A model of a Futoshiki grid built using only
      binary not-equal constraints for both the row and column constraints.

2. futoshiki_csp_model_2 (worth 20/100 marks)
    - A model of a Futoshiki grid built using only n-ary
      all-different constraints for both the row and column constraints.
    
    The input board is specified as a list of n lists. Each of the n lists
    represents a row of the board. If a 0 is in the list it represents an empty
    cell. Otherwise if a number between 1--n is in the list then this
    represents a pre-set board position.

    Each list is of length 2n-1, with each space on the board being separated
    by the potential inequality constraints. '>' denotes that the previous
    space must be bigger than the next space; '<' denotes that the previous
    space must be smaller than the next; '.' denotes that there is no
    inequality constraint.

    E.g., the board

    -------
    | > |2|
    | | | |
    | | < |
    -------
    would be represented by the list of lists

    [[0,>,0,.,2],
     [0,.,0,.,0],
     [0,.,0,<,0]]

'''
import cspbase
import itertools


def futoshiki_csp_model_1(futo_grid):
    '''Futoshiki model using only binary not-equal constraints.'''
    n = len(futo_grid)
    csp = cspbase.CSP("Futoshiki Model 1")
    
    # Create variables
    var_array = []
    for i in range(n):
        row_vars = []
        for j in range(0, 2*n-1, 2):
            domain = [futo_grid[i][j]] if futo_grid[i][j] != 0 else list(range(1, n+1))
            var = cspbase.Variable(f"V{i}{j//2}", domain)
            csp.add_var(var)
            row_vars.append(var)
        var_array.append(row_vars)
    
    # Add row and column binary not-equal constraints
    for i in range(n):
        for j in range(n):
            for k in range(j+1, n):
                constraint_row = cspbase.Constraint(f"C_R_{i}_{j}_{k}", [var_array[i][j], var_array[i][k]])
                satisfying_tuples = [(x, y) for x in var_array[i][j].cur_domain() for y in var_array[i][k].cur_domain() if x != y]
                constraint_row.add_satisfying_tuples(satisfying_tuples)
                csp.add_constraint(constraint_row)
                
                constraint_col = cspbase.Constraint(f"C_C_{j}_{i}_{k}", [var_array[j][i], var_array[k][i]])
                satisfying_tuples = [(x, y) for x in var_array[j][i].cur_domain() for y in var_array[k][i].cur_domain() if x != y]
                constraint_col.add_satisfying_tuples(satisfying_tuples)
                csp.add_constraint(constraint_col)
    
    # Add inequality constraints
    for i in range(n):
        for j in range(1, 2*n-1, 2):
            if futo_grid[i][j] == '>':
                constraint = cspbase.Constraint(f"C_I_{i}_{j//2}", [var_array[i][j//2], var_array[i][j//2+1]])
                satisfying_tuples = [(x, y) for x in var_array[i][j//2].cur_domain() for y in var_array[i][j//2+1].cur_domain() if x > y]
                constraint.add_satisfying_tuples(satisfying_tuples)
                csp.add_constraint(constraint)
            elif futo_grid[i][j] == '<':
                constraint = cspbase.Constraint(f"C_I_{i}_{j//2}", [var_array[i][j//2], var_array[i][j//2+1]])
                satisfying_tuples = [(x, y) for x in var_array[i][j//2].cur_domain() for y in var_array[i][j//2+1].cur_domain() if x < y]
                constraint.add_satisfying_tuples(satisfying_tuples)
                csp.add_constraint(constraint)
    
    return csp, var_array

def futoshiki_csp_model_2(futo_grid):
    '''Futoshiki model using n-ary all-different constraints.'''
    n = len(futo_grid)
    csp = cspbase.CSP("Futoshiki Model 2")
    
    # Create variables
    var_array = []
    for i in range(n):
        row_vars = []
        for j in range(0, 2*n-1, 2):
            domain = [futo_grid[i][j]] if futo_grid[i][j] != 0 else list(range(1, n+1))
            var = cspbase.Variable(f"V{i}{j//2}", domain)
            csp.add_var(var)
            row_vars.append(var)
        var_array.append(row_vars)
    
    # Add n-ary all-different constraints for rows and columns
    for i in range(n):
        constraint_row = cspbase.Constraint(f"C_Row_{i}", var_array[i])
        satisfying_tuples = [t for t in itertools.permutations(range(1, n+1), n)]
        constraint_row.add_satisfying_tuples(satisfying_tuples)
        csp.add_constraint(constraint_row)
        
        col_vars = [var_array[j][i] for j in range(n)]
        constraint_col = cspbase.Constraint(f"C_Col_{i}", col_vars)
        satisfying_tuples = [t for t in itertools.permutations(range(1, n+1), n)]
        constraint_col.add_satisfying_tuples(satisfying_tuples)
        csp.add_constraint(constraint_col)
    
    # Add inequality constraints
    for i in range(n):
        for j in range(1, 2*n-1, 2):
            if futo_grid[i][j] == '>':
                constraint = cspbase.Constraint(f"C_I_{i}_{j//2}", [var_array[i][j//2], var_array[i][j//2+1]])
                satisfying_tuples = [(x, y) for x in var_array[i][j//2].cur_domain() for y in var_array[i][j//2+1].cur_domain() if x > y]
                constraint.add_satisfying_tuples(satisfying_tuples)
                csp.add_constraint(constraint)
            elif futo_grid[i][j] == '<':
                constraint = cspbase.Constraint(f"C_I_{i}_{j//2}", [var_array[i][j//2], var_array[i][j//2+1]])
                satisfying_tuples = [(x, y) for x in var_array[i][j//2].cur_domain() for y in var_array[i][j//2+1].cur_domain() if x < y]
                constraint.add_satisfying_tuples(satisfying_tuples)
                csp.add_constraint(constraint)
    
    return csp, var_array
