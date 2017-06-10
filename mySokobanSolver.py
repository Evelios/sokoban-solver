from cab320_search import *
from cab320_sokoban import *

import copy

# - - - - - Global Variables - - - - - - - - - - - - - - - - - - - - - - - - -

UP    = ( 0, -1)
DOWN  = ( 0,  1)
LEFT  = (-1,  0)
RIGHT = ( 1,  0)

MOVE_DICT = {"Up"    : UP,
             "Down"  : DOWN,
             "Left"  : LEFT,
             "Right" : RIGHT}

# - - - - - Util Functions - - - - - - - - - - - - - - - - - - - - - - - - - -

def tupleAdd(t0, t1):
    return (t0[0] + t1[0], t0[1] + t1[1])

def tupleSubtract(t0, t1):
    return (t0[0] - t1[0], t0[1] - t1[1])

## Returns the non-tangential distance between two points
def manhattanDist(t0, t1):
    return abs(t0[0] - t1[0]) + abs(t0[1] - t1[1])

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class SokobanPuzzle(Problem):
    '''
    Class to represent a Sokoban puzzle.
    Your implementation should be compatible with the
    search functions of the module  cab320_search

    '''
    def __init__(self, puzzleFileName):
        
        ## Load up the warehouse
        warehouse = Warehouse()
        warehouse.read_warehouse_file(puzzleFileName)

        ## Load up the taboo cells
        tabooCellString = tabooCells(puzzleFileName).split("\n")
        tabooWarehouse = Warehouse()
        tabooWarehouse.extract_locations(tabooCellString)

        ## Store the puzzle variables
        self.walls = tuple(warehouse.walls)
        self.targets = tuple(warehouse.targets)
        ## this is applicable to box move check
        self.tabooWalls = tuple(tabooWarehouse.walls)

        ## Store the initial states as a tuple of the
        ## worker position and the box positions
        ## EG.// ( (1,3), ((4, 2), (3, 4) (2, 2)) )
        self.initial = (warehouse.worker, tuple(warehouse.boxes))
        
        ## Goal state is the end state of the boxes stored in a tuple
        ## by default the goal state will be the target locations
        self.goal = self.targets

    def actions(self, state):
        ''' Return actions that can be executed in a given state '''
        
        actions = []
        
        for command, direction in MOVE_DICT.iteritems():
            nextLoc = tupleAdd(state[0], direction)
            nextBoxLoc = tupleAdd(nextLoc, direction)
            ## Check if worker can move
            if nextLoc not in self.walls:
                ## Check if worker is trying to move a box
                if nextLoc in state[1]:
                    ## Check if worker can move box
                    if nextBoxLoc not in self.tabooWalls and \
                       nextBoxLoc not in state[1]:
                        actions.append(command)
                else:
                    actions.append(command)

        return actions

    def result(self, state, action):
        ''' Return the state that results from executing
        the given state. Actions must be one of
        self.actions(state). '''

        assert action in self.actions(state)

        ## New state is old state with worker moved by action
        workerPos = tupleAdd(state[0], MOVE_DICT[action])

        boxes = list(state[1])
        ## If worker is on a box move the box
        if workerPos in boxes:
            boxes.remove(workerPos)
            boxes.append(tupleAdd(workerPos, MOVE_DICT[action]))

        newState = (workerPos, tuple(boxes))

        return newState

    def goal_test(self, state):
        ''' Retrun true if that state is a goal.
        The goal state is if all the targets in state
        have boxes on them '''

        assert len(self.targets) == len(state[1])

        for box in state[1]:
            if box not in self.goal:
                return False

        return True
        
##        return self.goal == state[1]

    def path_cost(self, c, state1, action, state2):
        '''Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path.'''
        return c + 1

    def print_solution(self, goal_node):
        ''' Shows the representation of a solution by
        a specific goal node. Eg: goal can by found by
            goal_node = breadth_first_tree_search(problem
        '''
        # path is list of nodes from initial state (root of the tree)
        # to the goal_node
        path = goal_node.path()
        # print the solution
        print "Solution takes {0} steps from the initial state".format(len(path)-1)
        print path[0].state
        print "to the goal state"
        print path[-1].state
        print "\nBelow is the sequence of moves\n"
        print [node.action for node in path]           

    def h(self, node):
        return sokoban_heuristic(self, node)
            
    def get_solution(self, goal_node):
        ''' Get a list representation of the actions the worker takes to
            reach the goal node'''
        
        return [node.action for node in goal_node.path()[1:]]
    
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class SokobanMicro(Problem):
    def __init__(self, puzzleFileName):
        
        ## Load up the warehouse and get parameters
        warehouse = Warehouse()
        warehouse.read_warehouse_file(puzzleFileName)
        self.walls = tuple(warehouse.walls)

        ## Initialize other variables
        self.init(warehouse.worker, warehouse.worker, tuple(warehouse.boxes))

    def init(self, worker, target, boxes):
        self.initial = worker
        self.goal = target
        self.boxes = boxes

    def actions(self, state):

        actions = []

        for command, direction in MOVE_DICT.iteritems():
            new_pos = tupleAdd(state, direction)
            if new_pos not in self.walls and new_pos not in self.boxes:
                actions.append(command)

        return actions

    def result(self, state, action):
        ''' Get the new state representation from the starting state
            to another state after taking action'''
        
        return tupleAdd(state, MOVE_DICT[action])

    def goal_test(self, state):
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        ''' The path cost is simply the previous path cost plus one'''
        return c + 1

    def h(self, node):
        ''' The heuristic for the micro action is the manhattan
            distance of the worker from his target location in '''
        
        return manhattanDist(node.state, self.goal)

    def get_solution(self, goal_node):
        ''' Get a list representation of the actions the worker takes to
            reach the goal node'''

        return [node.action for node in goal_node.path()[1:]]


class SokobanMacro(Problem):

    def __init__(self,puzzleFileName):
        
        ## Load up the warehouse
        warehouse = Warehouse()
        warehouse.read_warehouse_file(puzzleFileName)

        ## Store the puzzle variables
        self.walls = tuple(warehouse.walls)
        self.targets = tuple(warehouse.targets)

        ## Load up the taboo cells
        ## this is applicable to box move check
        tabooCellString = tabooCells(puzzleFileName).split("\n")
        warehouse.extract_locations(tabooCellString)
        self.tabooWalls = tuple(warehouse.walls)

        ## Store the initial states as a tuple of the
        ## worker position and the box positions
        ## EG.// ( (1,3), ((4, 2), (3, 4) (2, 2)) )
        self.initial = (warehouse.worker, tuple(warehouse.boxes))
        
        ## Goal state is the end state of the boxes stored in a tuple
        ## by default the goal state will be the target locations
        self.goal = self.targets

        self.microProblem = SokobanMicro(puzzleFileName)
    
    def actions(self, state):
        ''' Return actions that can be executed in a given state
            An action is defined as the previous box location,
            its new location, and the goal node of micro actions there
            EG.// action = ( (1, 1), (1, 2), <node object> )'''

        actions = []

        worker, boxes = state

        for box in boxes:
            for command, direction in MOVE_DICT.iteritems():

                push_pos = tupleSubtract(box, direction)
                new_box_pos = tupleAdd(box, direction)

                ## If area in front of and behind the box are clear
                ## run search to see if worker can move to location
                if push_pos not in self.walls and push_pos not in boxes and \
                   new_box_pos not in self.tabooWalls and new_box_pos not in boxes:

                    ## Run Search
                    self.microProblem.init(worker, push_pos, boxes)
##                    result = iterative_deepening_astar(self.microProblem)
                    result = astar_search(self.microProblem)

                    if result:                        
                        actions.append( (box, command, result) )
        
        return actions

    def result(self, state, action):
        ''' Return the state that results from executing
        the given state. Actions must be one of
        self.actions(state). '''

        worker, boxes = state
        boxes = list(boxes)

        
        worker = action[0]
        moved_box = tupleAdd(action[0], MOVE_DICT[action[1]])

        boxes.remove(action[0])
        boxes.append(moved_box)
        boxes = tuple(boxes)

        return (worker, boxes)

    def goal_test(self, state):
        
        boxes = state[1]

        ## Check if all the boxes are in a goal state
        for box in boxes:
            if box not in self.goal:
                return False
        return True

    def path_cost(self, c, state1, action, state2):
        '''Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path.'''

        ## Path cost is equal to the number of moves the
        ## worker takes to move the box once (-1 for starting node)
        
        return c + len(action[2].path()) - 1

    def h(self, node):
        return sokoban_heuristic(self, node)

    def get_solution(self, goal_node):
        ''' Get a list representation of the actions the worker takes to
            reach the goal node'''

        solutions = []

        actions = [(node.action[1], self.microProblem.get_solution(node.action[2]))
                 for node in goal_node.path()[1:]]

        for macro, micro in actions:
            solutions.extend(micro)
            solutions.append(macro)

        return solutions

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def sokoban_heuristic(problem, node):
    ''' Heuristic for goal state.
        Heuristic is the sum of the minimum manhattan distance
        for all the boxes from their target at the current
        node in the problem and the manhattan distance
        from the worker to the closet box'''

    assert len(node.state[1]) == len(problem.targets)

    ## Search Space
    boxes = node.state[1]
    boxesList = list(boxes)
    targets = list(problem.targets)
    worker = node.state[0]

    ## Resulting heuristic
    total = 0

    while boxesList:
        minDistance = float('inf')
        minBox = boxesList[0]
        minTarget = targets[0]

        ## Find the target/box combo with the smallest distance
        for box in boxesList:
            for target in targets:
                distance = manhattanDist(box, target)
                if distance < minDistance:
                    minDistance = distance
                    minBox = box
                    minTarget = target

        ## Remove from search space and add heuristic
        boxesList.remove(minBox)
        targets.remove(minTarget)
        total += minDistance

    ## Find the minimum distance from the worker to a box
    minWorkerDistance = float('inf')
    
    for box in boxes:
        distance = manhattanDist(box, worker)
        if distance < minWorkerDistance:
            minWorkerDistance = distance
    total += minWorkerDistance
        
    return total
        
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def checkActions(puzzleFileName, actionSequence):
    '''
    This is a function called by the automatic marker.

    Your implementation should load a Sokoban puzzle from a text file,
    then try to apply the sequence of actions listed in actionSequence.

    @param puzzleFileName: file name of the puzzle
         (same format as for the files in the warehouses directory)
    @param actionSequence: a sequence of actions.
           For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
    @return:
        The string 'Failure', if one of the move was not successul.
           For example, if the agent tries to push two boxes,
                or push into to push into a wall.
        Otherwise, if all moves were successful return         
           A string representing the state of the puzzle after applying
           the sequence of actions.  This should be the same string as the
           string returned by the method  Warehouse.visualize()
    '''

    ## Load warehouse
    warehouse = Warehouse()
    warehouse.read_warehouse_file(puzzleFileName)

    ## Apply Sequence of actions
    for action in actionSequence:
        if not move_player(warehouse, action):
            return "Failure"
        
        
    ## If failure state not reached return string of solution state
    return warehouse.visualize()
        
def move_player(warehouse, direction):
    ''' Direction in ["Up", "Down", "Left", "Right"]
        Check if worker is pushing a box
        Return bool for succes of movement
    '''
    
    ## Current worker position
    worker = warehouse.worker

    ## Possible next worker position
    nextPos = tupleAdd(worker, MOVE_DICT[direction])

    ## Test is the next move is not possible
    if nextPos in warehouse.walls:
        return False
    ## If the worker is trying to move a box
    if nextPos in warehouse.boxes:
        nextBoxPos = tupleAdd(nextPos, MOVE_DICT[direction])
        if (nextBoxPos not in warehouse.walls and
            nextBoxPos not in warehouse.boxes):
            ## Can move box
            warehouse.boxes.remove(nextPos)
            warehouse.boxes.append(nextBoxPos)
        else:
            ## Box cannot be moved
            return False

    ## Worker can move so update his position
    warehouse.worker = nextPos
    return True

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def tabooCells(puzzleFileName):
    '''
    This is a function called by the automatic marker.

    Your implementation should load a Sokoban puzzle from a text file,
    then identify the cells that should be avoided in the sense that if
    a box get pushed on such a cell then the puzzle becomes unsolvable.

    @param puzzleFileName: file name of the puzzle
         (same format as for the files in the warehouses directory)
    @return:
           A string representing the puzzle with the taboo cells marked with an 'X'.
           Apart from the 'X's, the string should follows the same format as the
           string returned by the method  Warehouse.visualize()
    '''

    warehouse = Warehouse()
    warehouse.read_warehouse_file(puzzleFileName)

    ## Get warehouse height and width
    # this assumes that walls are surrounding the puzzle area
    height = 0
    width = 0
    for pos in warehouse.walls:
        if pos[0] > width:
            width = pos[0]
        if pos[1] > height:
            height = pos[1]

    ## Gather tabooCells
    tabooCells = []
    ''' Taboo cells include...
    Cells that have 2 or more adjacent wall
    neighbors, or 'corners'
    '''

    ## Calculate corner taboo cells
    cornerCells = []
    
    for x in range(width):
        for y in range(height):
            position = (x, y)
            adjWalls = []
            if  position not in warehouse.targets and \
                position not in warehouse.walls and \
                position not in warehouse.boxes:
                adjWalls = getAdjWalls(warehouse, position)
                
            ''' If adjCells is length 3 it is a bad square
            If adjCells is length 2, check if boxes make corner
            this means that x and y coordinates must both be different
            '''
            if len(adjWalls) >= 3 or (len(adjWalls) == 2 and \
                 adjWalls[0][0] != adjWalls[1][0] and \
                 adjWalls[0][1] != adjWalls[1][1]):
                cornerCells.append(position)

    ## Calculate taboo cells between corners
    tabooBetweenCorner = []

    ## Loop for both the x and y
    for i in range(2):
        ## j is the other coordingate for checking
        j = abs(i - 1)

        ## Loop through comparing all combinations of corner
        ## Cells checking for more taboo cells inbetween
        for q in range(len(cornerCells)):
            for w in range(q, len(cornerCells)):
                cell1 = cornerCells[q]
                cell2 = cornerCells[w]
                
                ## Don't compare if cells are the same
                if cell1 != cell2 and cell1[i] == cell2[i]:

                    ## Loop through cells inbetween two cells
                    distance = cell2[j] - cell1[j]                    
                    middleCells = []
                    for d in range(1, distance):
                        checkedCell = list(cell1)
                        checkedCell[j] += d

                        ## Used for checking if the two targets
                        ## run along a wall
                        adjWallCheck = []
                        one = copy.copy(checkedCell)
                        one[i] += 1
                        one = tuple(one)
                        
                        two = copy.copy(checkedCell)
                        two[i] -= 1
                        two = tuple(two)
                        
                        checkedCell = tuple(checkedCell)
                        
                        if checkedCell in warehouse.targets or \
                           (one not in warehouse.walls and two not in warehouse.walls):
                            middleCells = []
                            break
                        else:
                            middleCells.append(checkedCell)

                    ## Add middle cells to non corner taboo cells
                    tabooBetweenCorner.extend(middleCells)

    ## Add cells and return as walls
    tabooCells.extend(cornerCells)
    tabooCells.extend(tabooBetweenCorner)
    warehouse.walls.extend(tabooCells)
    return warehouse.visualize()

def getAdjWalls(warehouse, position):
    adjWalls = []
    for direction in MOVE_DICT:
        adjPosition = tupleAdd(position, MOVE_DICT[direction])
        if adjPosition in warehouse.walls:
            adjWalls.append(adjPosition)
    return adjWalls

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def solveSokoban_elementary(puzzleFileName, timeLimit = None):
    '''
    This is a function called by the automatic marker.

    This function should solve the puzzle defined in a file.

    @param puzzleFileName: file name of the puzzle
         (same format as for the files in the warehouses directory)
    @param time_limit: The time limit for this agent in seconds .
    @return:
        A list of strings.
        If timeout return  ['Timeout']
        If puzzle cannot be solved return ['Impossible']
        If a solution was found, return a list of elementary actions that solves
        the given puzzle coded with 'Left', 'Right', 'Up', 'Down'
        For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
        If the puzzle is already in a goal state, simply return []
    '''
    
    puzzle = SokobanPuzzle(puzzleFileName)
    result = None
    
    if timeLimit:
##        result = astar_search(puzzle)
        result = iterative_deepening_astar(puzzle, timeLimit)
    else:
##        result = astar_search(puzzle)
        result = iterative_deepening_astar(puzzle)
    if result:
        if result == 'cutoff':
            return ['Timeout']
        else:
            return puzzle.get_solution(result)
    else:
        return ['Impossible']

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def solveSokoban_macro(puzzleFileName, timeLimit = None):
    '''
    This is a function called by the automatic marker.
    
    This function has the same purpose as 'solveSokoban_elementary', but 
    it should internally use macro actions as suggested 
    in the assignment description. Although it internally uses macro 
    actions, this function should return a sequence of 
    elementary  actions.


    @param puzzleFileName: file name of the puzzle
         (same format as for the files in the warehouses directory)
    @param time_limit: The time limit for this agent in seconds .
    @return:
        A list of strings.
        If timeout return  ['Timeout']
        If puzzle cannot be solved return ['Impossible']
        If a solution was found, return a list elementary actions that solves
        the given puzzle coded with 'Left', 'Right', 'Up', 'Down'
        For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
        If the puzzle is already in a goal state, simply return []
    '''

    puzzle = SokobanMacro(puzzleFileName)
    result = None
    
    if timeLimit:
        result = iterative_deepening_astar(puzzle, timeLimit)
##        result = astar_search(puzzle)
    else:
        result = iterative_deepening_astar(puzzle)
##        result = astar_search(puzzle)

    if result:
        if result == 'cutoff':
            return ['Timeout']
        else:
            return puzzle.get_solution(result)
    else:
        return ['Impossible']

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

