#from asyncio.windows_events import NULL
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
#from collections import deque

class Dikjstra:
    
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.visited = []
        self.q = {}
        self.parent = {}
        self.reached = 0
        self.animationGrid = self.createAnimationGrid()
        self.computationGrid = self.createComputationalGrid(self.animationGrid)

    def createAnimationGrid(self):
        grid = np.ones([250, 400, 3],dtype=np.uint8)
        contours1 = np.array([[36,65], [115,40], [80,70], [105,150]])
        contours2 = np.array([[165,130], [165,180], [200,200], [235,180], [235,130], [200,110]])
        cv2.fillPoly(grid, pts = [contours1], color =(255,255,0))
        cv2.fillPoly(grid, pts = [contours2], color =(255,255,0))
        cv2.circle(grid, (300, 65), 40,color =(255,255,0), thickness = -1)
        # plt.imshow(grid, plt.cm.gray)
        # plt.show()
        return grid
    
    def createComputationalGrid(self, animGrid):
        grid = np.ones([250, 400])
        for i in range (250):
            for j in range (400):
                if animGrid[i, j][2] == 0:
                    grid[i, j] = -1
                else :
                    grid[i, j] = math.pow(10, 15)
        return grid

    def isValid(self, i, j, mainMat):             
        if 0 <= i < 250 and 0 <= j < 400 and mainMat[i, j] != -1 :
            if self.visited.count((i, j)) == 0:
                return True
        return False
    
    def findSubordinates(self, mainMat, coord):
        i = coord[0]
        j = coord[1]
        cost = [1, 1.4, 1, 1.4, 1, 1.4, 1, 1.4]
        count = 0
        self.visited.append((i, j))
        for data in [[-1,0], [-1,1], [0,1], [1,1], [1,0], [1,-1], [0,-1], [-1,-1]]:
            new_i = i + data[1]
            new_j = j + data[0]
            if(self.isValid(new_i, new_j, mainMat)):
                if not((new_i, new_j) in self.q) or mainMat[new_i, new_j] == math.pow(10, 15):
                    mainMat[new_i, new_j] = min(mainMat[i, j] + cost[count], mainMat[new_i, new_j])
                    self.parent[(new_i, new_j)] = (i,j) 
                    self.animationGrid[new_i, new_j] = [0, 0, 255]
                    self.q[(new_i, new_j)] = mainMat[new_i, new_j]
                    if(new_i, new_j) == self.goal:
                        print("Goal Reached")
                        self.reached = 1
                        return 
                else :
                    mainMat[new_i, new_j] = min(mainMat[i, j] + cost[count], mainMat[new_i, new_j])
                    if(mainMat[new_i, new_j] > mainMat[i, j] + cost[count]): 
                        self.parent[(new_i, new_j)] = (i,j) 
                        self.q[(new_i, new_j)] = mainMat[new_i, new_j]
                    if(new_i, new_j) == self.goal:
                        print("Goal Reached")
                        self.reached = 1
                        return 
                    self.animationGrid[new_i, new_j] = [0, 0, 255]
            count += 1 
        
    def dikjstra(self):
        mainMat = self.computationGrid
        mainMat[self.start[1], self.start[0]] = 0
        self.q[self.start] = 0
        self.parent[self.start] = self.start
        while(len(self.q) != 0):
            coord = min(self.q, key=self.q.get)
            self.q.pop(coord, None)
            self.findSubordinates(mainMat, coord)
            if self.reached == 1:
                self.backTrack(self.parent)
                break
            # break
            # cv2.imshow("frame", self.animationGrid)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     return 0
    def backTrack(self, parent):
        coord = self.goal
        while(parent[coord] != self.start):
            coord = parent[coord]
            print((coord[0], coord[1]))
            self.animationGrid[coord[0], coord[1]] = [0, 255, 0]
        cv2.imshow("frame", self.animationGrid)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            return 0

start = (0, 0)
goal = (125, 300)
p1 = Dikjstra(start, goal)
p1.dikjstra()
