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

    def lineEquation(self, x, y, x1, y1, x2, y2, decide):
        if decide == 0:
            return (y - y1) - ((y2 - y1)/(x2 - x1)*(x - x1)) >= 0
        else :
            return (y - y1) - ((y2 - y1)/(x2 - x1)*(x - x1)) <= 0

    def getSlope(self,x1, y1, x2, y2):
        A = -(y2 - y1)/(x2 - x1)
        B = 1
        D = 5
        return 5*(math.pow(math.pow(A,2) + math.pow(B,2), 0.5))

    def createAnimationGrid(self):
        grid = np.ones([250, 400, 3],dtype=np.uint8)
        rows, cols = 250, 400
        r = 40
        center = [65, 300]
        x, y = np.ogrid[:rows, :cols]
        mask_area = (x - center[0]) ** 2 + (y - center[1]) ** 2 <= r*r
        mask_area_green = (x - center[0]) ** 2 + (y - center[1]) ** 2 <= 45*45
        # print(mask_area)
        grid[mask_area_green] = [0, 255, 0]
        grid[mask_area] = [0, 255, 255]
        
        for i in range(rows):
            for j in range(cols):
                
                if 0 <= j < 5 or 394 <= j < 400 or 0 <= i < 5 or 245 <= i < 250:
                    grid[i, j] = [0, 255, 0]

                if self.lineEquation(j, i, 36 - 3, 65 -3, 115, 40-3, 0) and self.lineEquation(j, i, 36 - 3, 65 + 3, 105, 150 + 5, 1) and (self.lineEquation(j, i, 115 + 4, 40, 80 + 4, 70, 1) or self.lineEquation(j, i, 80+4, 70, 105+3, 150, 0)):
                    grid[i, j] = [0, 255, 0]

                if self.lineEquation(j, i, 200, 104, 163, 125, 0) and self.lineEquation(j, i, 201, 104, 238, 125, 0) and 160 <= j <= 240 and (self.lineEquation(j, i, 201, 206, 163, 185, 1) and self.lineEquation(j, i, 200, 205, 237, 185, 1)):
                    grid[i, j] = [0, 255, 0]
        
        for i in range(rows):
            for j in range(cols):
                if self.lineEquation(j, i, 36, 65, 115, 40, 0) and self.lineEquation(j, i, 36, 65, 105, 150, 1) and (self.lineEquation(j, i, 115, 40, 80, 70, 1) or self.lineEquation(j, i, 80, 70, 105, 150, 0)):
                    grid[i, j] = [0, 255, 255]

                if self.lineEquation(j, i, 200, 110, 165, 130, 0) and self.lineEquation(j, i, 200, 110, 235, 130, 0) and 165 <= j <= 235 and (self.lineEquation(j, i, 200, 200, 165, 180, 1) and self.lineEquation(j, i, 200, 200, 235, 180, 1)):
                    grid[i, j] = [0, 255, 255]
                
        # contours1 = np.array([[36,65], [115,40], [80,70], [105,150]])
        # contours2 = np.array([[165,130], [165,180], [200,200], [235,180], [235,130], [200,110]])
        # cv2.fillPoly(grid, pts = [contours1], color =(255,255,0))
        # cv2.fillPoly(grid, pts = [contours2], color =(255,255,0))
        # cv2.circle(grid, (300, 65), 40,color =(255,255,0), thickness = -1)
        # plt.imshow(grid, plt.cm.gray)
        # plt.show()
        return grid
    
    def createComputationalGrid(self, animGrid):
        grid = np.ones([250, 400])
        for i in range (250):
            for j in range (400):
                if animGrid[i, j][0] == 0:
                    grid[i, j] = -1
                else :
                    grid[i, j] = math.pow(10, 15)
        return grid

    def checkValidity(self, i , j):
        if self.animationGrid[i, j][2] == 0:
            print(" incorrect Coordinates")
            exit()

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
                    if(mainMat[new_i, new_j] > mainMat[i, j] + cost[count]): 
                        self.parent[(new_i, new_j)] = (i,j) 
                        self.q[(new_i, new_j)] = mainMat[new_i, new_j]
                    mainMat[new_i, new_j] = min(mainMat[i, j] + cost[count], mainMat[new_i, new_j])
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
            cv2.imshow("frame", self.animationGrid)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return 0
                
    def backTrack(self, parent):
        coord = self.goal
        while(parent[coord] != self.start):
            coord = parent[coord]
            #print((coord[0], coord[1]))
            self.animationGrid[coord[0], coord[1]] = [255, 0, 0]
            cv2.imshow("frame", self.animationGrid)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                return 0

start_x = int(input ("Enter start x coordinate :"))
start_y = int(input ("Enter start y coordinate :"))
goal_x = int(input ("Enter goal x coordinate :"))
goal_y = int(input ("Enter goal y coordinate :"))

start = (start_y, start_x)
goal = (goal_y, goal_x)
p1 = Dikjstra(start, goal)
p1.checkValidity(start_x , start_y)
p1.checkValidity(goal_x , goal_y)
p1.dikjstra()
