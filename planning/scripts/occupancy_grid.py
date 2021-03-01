#!/usr/bin/env python

from __future__ import print_function
import sys
import json 
import math

from numpy.core.defchararray import array
sys.path.append('/home/karl/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/') 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from path_planning import pathPlanning

class OccupancyGrid():
    def __init__(self, filePath, gridSize = 1):
        self.filePath = filePath
        self.gridSize = gridSize
        self.grid = None
        self.world = None
        self.xLim = None
        self.yLim = None
        self.xMax = 0 
        self.yMax = 0
    
    def readWorld(self):
        with open(self.filePath, 'rb') as f:
            self.world = json.load(f) 

    def setGrid(self):
        airspace = self.world['airspace']
        self.xLim = (int(airspace['min'][0] * self.gridSize), int(airspace['max'][0] *self.gridSize))
        self.yLim = (int(airspace['min'][1] * self.gridSize), int(airspace['max'][1] *self.gridSize))
        
        self.xMax = self.xLim[1] - self.xLim[0]
        self.yMax = self.yLim[1] - self.yLim[0]

        self.grid = np.zeros((self.yMax, self.xMax))

    def drawWalls(self):
        walls = self.world['walls']
        if len(walls) == 0:
            print("no walls in this map")
            return 
        
        for wall in walls:
            start = wall['plane']['start']
            stop = wall['plane']['stop']

            startX, endX = int(start[0] * self.gridSize - self.xLim[0]), int(stop[0] * self.gridSize - self.xLim[0])
            startY, endY = int(start[1] * self.gridSize - self.yLim[0]), int(stop[1] * self.gridSize - self.yLim[0])

            if startX >= self.xMax:
                startX = self.xMax -1

            if endX >= self.xMax:
                endX = self.xMax -1

            if startY >= self.yMax:
                startY = self.yMax -1

            if endY >= self.yMax:
                endY = self.yMax -1


            if (endX - startX) != 0:
                k = (endY - startY)/(endX - startX)
                m = startY - int(k) * startX
                
                if startX < endX:
                    for i in range(startX, endX):
                        if startX - self.xLim[1] < 0 and self.gridSize == 1:
                            m = m - 1
                        self.grid[k*i + m, i] = 1
                        #self.grid[k*i + m -1, i] = 2
                        #self.grid[k*i + m +1, i] = 2
                        #self.grid[k*i + m, i-1] = 2
                        #self.grid[k*i + m, i+1] = 2
                        if k*i + m -1 >= 0 and k*i + m - 1 <= self.yMax -1:
                            self.grid[k*i + m -1, i] = 2

                        if k*i + m + 1 >= 0 and k*i + m + 1 <= self.yMax -1:
                            self.grid[k*i + m +1, i] = 2
                        
                        if i + 1 >= 0 and i + 1 <= self.xMax-1:
                            self.grid[k*i + m, i-1] = 2
                        
                        if i + 1 >= 0 and i + 1 <= self.xMax-1:
                            self.grid[k*i + m, i+1] = 2

                else:
                    for i in range(endX, startX):
                        if startX - self.xLim[1] < 0 and self.gridSize == 1:
                            m = m + 1
                        self.grid[k*i + m, i] = 1
                        if k*i + m -1 >= 0 and k*i + m - 1 <= self.yMax-1:
                            self.grid[k*i + m -1, i] = 2

                        if k*i + m + 1 >= 0 and k*i + m + 1 <= self.yMax-1:
                            self.grid[k*i + m +1, i] = 2
                        
                        if i + 1 >= 0 and i + 1 <= self.xMax -1:
                            self.grid[k*i + m, i-1] = 2
                        
                        if i + 1 >= 0 and i + 1 <= self.xMax-1:
                            self.grid[k*i + m, i+1] = 2

            else:
                if startY < endY:
                    for i in range(startY, endY):
                        self.grid[i, startX] = 1
                        if i - 1 >= 0 and i - 1 <= self.yMax -1:
                            self.grid[i-1, startX] = 2
                        if i + 1 >= 0 and i + 1 <= self.yMax -1:
                            self.grid[i+1, startX] = 2
                        if startX - 1 >= 0 and startX -1 <= self.xMax- 1:
                            self.grid[i, startX-1] = 2
                        if startX + 1 >= 0 and startX +1 <= self.xMax -1:
                            self.grid[i, startX+1] = 2
                else:
                    for i in range(endY, startY):
                        self.grid[i, startX] = 1
                        #self.grid[i-1, startX] = 2
                        #self.grid[i+1, startX] = 2
                        #self.grid[i, startX-1] = 2
                        #self.grid[i, startX+1] = 2
                        if i - 1 >= 0 and i - 1 <= self.yMax - 1:
                            self.grid[i-1, startX] = 2
                        if i + 1 >= 0 and i + 1 <= self.yMax - 1:
                            self.grid[i+1, startX] = 2
                        if startX - 1 >= 0 and startX -1 <= self.xMax - 1:
                            self.grid[i, startX-1] = 2
                        if startX + 1 >= 0 and startX +1 <= self.xMax - 1:
                            self.grid[i, startX+1] = 2


    def drawGate(self):
        gates = self.world['gates']
        if len(gates) == 0:
            print("no gates")
            return

        for gate in gates:
            x, y = int(gate['position'][0]), int(gate['position'][1])

            if y == 0:
                scaledX = x*self.gridSize - self.xLim[0]
                scaledY = - self.yLim[0]
                self.grid[scaledY - self.gridSize: scaledY + self.gridSize, \
                scaledX - 1: scaledX + 1] = 1

            if x == 0:
                scaledX =  - self.xLim[0]
                scaledY = y*self.gridSize - self.yLim[0]
                self.grid[scaledY - 1: scaledY + 1, \
                scaledX - self.gridSize: scaledX + self.gridSize] = 1


def main(argv=sys.argv):
    path = str(sys.path[-1]) + str(argv[1]) + ".world.json"
    gridSize = 3
    OG = OccupancyGrid(path, gridSize = gridSize)

    OG.readWorld()
    OG.setGrid()
    OG.drawWalls()
    OG.drawGate()

    start = (0 - OG.xLim[0], 10*gridSize)
    goal = (8*gridSize - OG.xLim[0], 10*gridSize)
    goal1 = (-OG.xLim[0], 5*gridSize)
    goal2 = (10, 30)
    planning = pathPlanning(OG.grid)
    planning.Astar(np.array(start), np.array(goal))
    planning.extractPath()
    print(planning.path)

    #arr = planning.extractPath()
    #for setpoint in arr: 
        


    cmap = colors.ListedColormap(["white", "blue"])
    plt.figure(figsize=(10,10))
    plt.imshow(OG.grid, cmap=cmap)

    plt.plot([p[0] for p in planning.path], [p[1] for p in planning.path], 'k', linewidth=4)
    planning.Astar(np.array(goal), np.array(goal1))
    planning.extractPath()
    plt.plot([p[0] for p in planning.path], [p[1] for p in planning.path], 'k', linewidth=4)
    planning.Astar(np.array(goal1), np.array(goal2))
    planning.extractPath()
    plt.plot([p[0] for p in planning.path], [p[1] for p in planning.path], 'k', linewidth=4)
    planning.Astar(np.array(goal2), np.array(start))
    planning.extractPath()
    plt.plot([p[0] for p in planning.path], [p[1] for p in planning.path], 'k', linewidth=4)


    plt.show()

if __name__ == "__main__":
    main()


