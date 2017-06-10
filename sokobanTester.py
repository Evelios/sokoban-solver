from cab320_sokoban import Warehouse
from mySokobanSolver import *
import time

if __name__ == "__main__":
##    problem_file = "./warehouses/warehouse_test.txt"
##    problem_file = "./warehouses/warehouse_test2.txt"  
    problem_file = "./warehouses/warehouse_03.txt"

    '''
    Display Problem
    '''

    field = Warehouse()
    field.read_warehouse_file(problem_file)
    print("\nProblem:")
    print field.visualize()

    '''
    Show Taboo Cells
    '''
    result = tabooCells(problem_file)
    print("\nTabooCells:")
    print(result)

    '''
    SolveProblem
    '''
    timeout = 600

    t0 = time.time()
    
##    result = solveSokoban_elementary(problem_file, timeout)
##    result = solveSokoban_elementary(problem_file)
    result = solveSokoban_macro(problem_file, timeout)
    
    t1 = time.time()
    
    print "\nSolution took ", t1-t0, " seconds\n"
    print result

    if len(result) > 0 and \
       result[0] != 'Impossible' and result[0] != 'Timeout':
        print checkActions(problem_file, result)
    
