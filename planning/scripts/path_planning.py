#!/usr/bin/env python
from __future__ import print_function
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm, colors
import random

from numpy import linalg
from numpy.linalg.linalg import norm



class AstarNode():
    def __init__(self, position, parent = None):
        self.position = position
        self.parent = parent

        self.g = 0
        self.h = 0
        self.f = 0
    
class minHeap():
    def __init__(self):
        self.heap = []

    def __str__(self):
        return ' '.join([str(i) for i in self.heap])
    
    def insert(self, data):
        self.heap.append(data)
    
    def checkFor(self, coordinates):
        for n in self.heap:
            if list(n.position) == list(coordinates):
                return n
        return False

    def isEmpty(self):
        if len(self.heap) == 0:
            return True

    def deleteMin(self):
        try:
            min = 0
            for i in range(len(self.heap)):
                if self.heap[i].f < self.heap[min].f:
                    min = i
            item = self.heap[min]
            del self.heap[min]
            return item 
        except IndexError:
            print()
            exit()


class pathPlanning():
    def __init__(self, grid):
        self.grid = grid
        self.xMin, self.yMin = 0, 0
        self.xMax, self.yMax = len(self.grid[0]) - 1, len(self.grid) - 1
        self.movements = np.array([(1, 0), (0,1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)])

    def __str__(self):
        return '\n'.join([str(row) for row in self.grid])

    def Astar(self, start, goal):
        startNode = AstarNode(start)
        goalNode = AstarNode(goal)

        openList = minHeap()
        openList.insert(startNode)
        closedList = []

        while not openList.isEmpty():
            currentNode = openList.deleteMin()
            
            for movement in self.movements:
                successorPosition = currentNode.position + movement
                if successorPosition[0] < self.xMin or successorPosition[0] > self.xMax or successorPosition[1] < self.yMin or successorPosition[1] > self.yMax:
                    continue

                if self.grid[successorPosition[0]][successorPosition[1]] == 1:
                    continue
                
                successorNode = AstarNode(successorPosition, parent=currentNode)
                if list(successorNode.position) == list(goalNode.position):
                    print("done")
                
                successorNode.g = currentNode.g + np.linalg.norm(movement)
                successorNode.h = max(abs(goalNode.position[0] - successorNode.position[0]), abs(goalNode.position[1] - successorNode.position[1]))
                successorNode.f = successorNode.g + successorNode.h 
                
                inOpenList = openList.checkFor(successorNode.position)
                if not inOpenList:
                    print("hej")
                    #if successorNode.g < inOpenList.g:
                    #    inOpenList.g = successorNode.g
                    #    inOpenList.f = inOpenList.g + successorNode.h
                    #    inOpenList.parent = successorNode.parent
                
                    #for closed in closedList:
                    #if list(closed.position) == list(successorNode.position):
                    #    if closed.f > successorNode.f:
                    #        openList.insert(successorNode)
                    
                closedList.append(currentNode)
                print(openList)

                



                                

            




def main():


    data = np.array([[0, 0, 0, 0, 0, 0, 3],
            [0, 0, 0, 1, 0, 0, 0],
            [2, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0]])

    #print(data.shape)


    planning = pathPlanning(data)
    start = np.array((0, 2))
    #print(type(start))
    goal = np.array((6, 0))
    #print(start + goal)
    planning.Astar(start=start, goal=goal)


    cmap = colors.ListedColormap(["white", "black", "green", "blue"])

    plt.figure(figsize=(10,10))
    plt.imshow(data, cmap=cmap)
    plt.axis('on')

    #plt.plot([-0.5, 6.5], [-0.5, -0.5], 'r', linewidth=2)


    plt.show()


if __name__ == "__main__":
    main()