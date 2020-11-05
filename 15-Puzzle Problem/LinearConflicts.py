#!/usr/bin/env python3
import time
import numpy
from heapq import heappop, heappush # See heapq_test.py file to learn how to use. Visit : https://docs.python.org/3/library/heapq.html

# Heuristic used: Manhattan Distance + 2*Linear Conflicts

# Heap to store the states
priorityQueue = []

# Dictionary to store coordinates for the goal state
finalPositions = {}

# Dictionary to store the states that have already been explored
explored = {}

def ConvertPath(path):      # Function to convert path from string to list, like "LRUD" to ["Left", "Right", "Up", "Down"]
    minPath = []
    for i in path:
        if i == 'L':
            minPath.append('Left')
        elif i == 'R':
            minPath.append('Right')
        elif i == 'U':
            minPath.append('Up')
        else:
            minPath.append('Down')
    return minPath

def LinearConflict(curState):           # To calculate the number of linear conflicts in a state
    linConflict = 0
    for i in range(4):
        for j in range(4):
            if curState[i*4 + j] == '0': 
                continue
            goal = finalPositions[curState[i*4 + j]]
            if (i, j) == goal: 
                continue
            if i == goal[0]:        # if correct row
                for col in range(j+1, 4):
                    if curState[i*4 + col] == '0':
                        continue
                    goalSwap = finalPositions[curState[i*4 + col]]
                    if goalSwap[0] == i and goal[1] > goalSwap[1]:
                        linConflict += 1
            elif j == goal[1]:      # if correct column
                for row in range(i+1, 4):
                    if curState[row*4 + j] == '0':
                        continue
                    goalSwap = finalPositions[curState[row*4 + j]]
                    if goalSwap[1]==j and goal[0]>goalSwap[0]:
                        linConflict += 1
    return linConflict

def Move(curStateInfo):
    nodesGenerated=0

    curState = curStateInfo[1]
    f = curStateInfo[0] - curStateInfo[4]
    minPath = curStateInfo[2]
    zeroPos = curStateInfo[3]

    try:
        lastMove = minPath[-1]
    except:
        lastMove = 'N'      # For the first move
    
    zx = zeroPos//4         # Row for blank
    zy = zeroPos%4          # Column for blank

    # Right
    if zy < 3 and lastMove!='L':    # swap zx, zy with zx, zy+1
        nodesGenerated+=1
        newState = curState[0:(zx*4 + zy)] + curState[(zx*4 + zy + 1)] + '0' + curState[(zx*4 + zy + 2):None]
        if newState not in explored:
            final_sx, final_sy = finalPositions[newState[zx*4 + zy]]

            # Calculate change in Manhattan distance heuristic using only the swapped tile
            dh = abs(zy - final_sy) - abs(zy + 1 - final_sy)

            # Recalculate the linear conflicts for new state
            linConf = 2*LinearConflict(newState)
            heappush(priorityQueue, [f + 1 + dh + linConf, newState, minPath+'R', 4 * zx + zy + 1, linConf])

    # Left
    if zy > 0 and lastMove!='R':    # swap zx, zy with zx, zy-1
        nodesGenerated+=1
        newState = curState[0:(zx*4 + zy - 1)] + '0' + curState[(zx*4 + zy - 1)] + curState[(zx*4 + zy + 1):None]
        if newState not in explored:
            final_sx, final_sy = finalPositions[newState[zx*4 + zy]]

            # Calculate change in Manhattan distance heuristic using only the swapped tile
            dh = abs(zy - final_sy) - abs(zy - 1 - final_sy)

            # Recalculate the linear conflicts for new state
            linConf = 2*LinearConflict(newState)
            heappush(priorityQueue, [f + 1 + dh + linConf, newState, minPath+'L', 4 * zx + zy - 1, linConf])

    # Up
    if zx > 0 and lastMove!='D':    # swap zx, zy with zx-1, zy
        nodesGenerated+=1
        newState = curState[0:((zx-1)*4 + zy)] + '0' + curState[((zx-1)*4 + zy + 1):(zx*4 + zy)] + curState[((zx-1)*4 + zy)] + curState[(zx*4 + zy + 1):None]
        if newState not in explored:
            final_sx, final_sy = finalPositions[newState[zx*4 + zy]]

            # Calculate change in Manhattan distance heuristic using only the swapped tile
            dh = abs(zx - final_sx) - abs(zx - 1 - final_sx)

            # Recalculate the linear conflicts for new state
            linConf = 2*LinearConflict(newState)
            heappush(priorityQueue, [f + 1 + dh + linConf, newState, minPath+'U', 4 * (zx - 1) + zy, linConf])

    # Down
    if zx < 3 and lastMove!='U':    # swap zx, zy with zx+1, zy
        nodesGenerated+=1
        newState = curState[0:(zx*4 + zy)] + curState[((zx+1)*4 + zy)] + curState[(zx*4 + zy + 1):((zx+1)*4 + zy)] + '0' + curState[((zx+1)*4 + zy + 1):None]
        if newState not in explored:
            final_sx, final_sy = finalPositions[newState[zx*4 + zy]]

            # Calculate change in Manhattan distance heuristic using only the swapped tile
            dh = abs(zx - final_sx) - abs(zx + 1 - final_sx)

            # Recalculate the linear conflicts for new state
            linConf = 2*LinearConflict(newState)
            heappush(priorityQueue, [f + 1 + dh + linConf, newState, minPath+'D', 4 * (zx + 1) + zy, linConf])

    return nodesGenerated

def FindMinimumPath(initialState,goalState):
    minPath=[]          # This list contains the sequence of actions in the optimal solution
    nodesGenerated=0    # This variable contains the number of nodes that were generated while finding the optimal solution
    
    for i in range(4):
        for j in range(4):
            finalPositions[goalState[i][j]] = [i, j]
    
    curState = ''
    for row in initialState:
        for i in row:
            curState += i
    path = ''
    
    zeroPos = 0
    for i in range(len(curState)):
        if curState[i] == '0':
            zeroPos = i
            break
    
    h = 0       
    for i in range(4):
        for j in range(4):
            tile = curState[i*4 + j]
            if tile=='0':
                continue
            goal = finalPositions[tile]
            h += abs(goal[0] - i) + abs(goal[1] - j)        # Manhattan distance
    linConf = 2*LinearConflict(curState)
    curStateInfo = [h + linConf, curState, path, zeroPos, linConf]

    while h!=0:
        nodesGenerated += Move(curStateInfo)
        curStateInfo = heappop(priorityQueue)
        while True:
            curState = curStateInfo[1]
            if curState not in explored:
                break
            curStateInfo = heappop(priorityQueue)
        path = curStateInfo[2]
        explored[curState] = True
        h = curStateInfo[0] - len(path) - curStateInfo[4]

    minPath = ConvertPath(path)
    # The variable path contains the encoded path as a string, like 'URDDL'
    # For that path, minPath = ['Up','Right','Down','Down','Left']
    
    return minPath, nodesGenerated


def ReadInitialState():
    with open("Test cases/initial_state1.txt", "r") as file: 
        initialState = [[x for x in line.split()] for i,line in enumerate(file) if i<4]
    return initialState

def ShowState(state,heading=''):
    print(heading)
    for row in state:
        print(*row, sep = " ")

def main():
    initialState = ReadInitialState()
    ShowState(initialState,'Initial state:')
    goalState = [['0','1','2','3'],['4','5','6','7'],['8','9','A','B'],['C','D','E','F']]
    ShowState(goalState,'Goal state:')
    
    start = time.time()
    minimumPath, nodesGenerated = FindMinimumPath(initialState,goalState)
    timeTaken = time.time() - start
    
    if len(minimumPath)==0:
        minimumPath = ['Up','Right','Down','Down','Left']
        print('Example output:')
    else:
        print('Output:')

    print('   Minimum path cost : {0}'.format(len(minimumPath)))
    print('   Actions in minimum path : {0}'.format(minimumPath))
    print('   Nodes generated : {0}'.format(nodesGenerated))
    print('   Time taken : {0} s'.format(round(timeTaken,4)))

if __name__=='__main__':
    main()
