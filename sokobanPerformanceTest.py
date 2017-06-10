from cab320_sokoban import Warehouse
from mySokobanSolver import *
import time
import os

if __name__ == "__main__":

    file = open('Results.txt', 'w+')

    for i in range(1, 205, 2):
    
        problem_file = "./warehouses/warehouse_" + str(i).zfill(2) + ".txt"

        '''
        SolveProblem
        '''
        timeout = 600

        t0 = time.time()
        
    ##    result = solveSokoban_elementary(problem_file, timeout)
    ##    result = solveSokoban_elementary(problem_file)
        result = solveSokoban_macro(problem_file, timeout)
        
        t1 = time.time()

        delta = t1-t0
        
        print "Puzzle ", i, " took ", delta, " seconds"

        file.write(str(i) + '\t' + str(delta) + '\n')
        file.flush()
        os.fsync(file)
##        print result
##
##        if len(result) > 0 and \
##           result[0] != 'Impossible' and result[0] != 'Timeout':
##            print checkActions(problem_file, result)
    
