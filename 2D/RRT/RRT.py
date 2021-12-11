import pygame
from RRT.RRTbasepy import RRTMap
from RRT.RRTbasepy import RRTGraph
import parameters
from pygame.locals import *
import sys
import math
import time 

class RRTMain:

    def __init__(self,windowWidth,windowHeight, start, goal, obstacles , n_obstacles):
        self.dimensions = (windowWidth,windowHeight)
        self.start_pos = start
        self.end_pos = goal
        self.n_obstacles = n_obstacles
        self.obstacles = obstacles
        self.obs_dim = parameters.OBSTACLE_DIM
        self.distance = 0
        self.time = 0

    def run(self):
        pygame.init()
        map=RRTMap(self.start_pos,self.end_pos,self.dimensions,self.obs_dim,self.n_obstacles)
        graph = RRTGraph(self.start_pos, self.end_pos, self.dimensions, self.obs_dim, self.n_obstacles)

        obstacles=graph.makeobs(self.obstacles)
        map.drawMap(obstacles)

        start_time = time.time()
        while not graph.path_to_goal():

            x, y, parent = graph.expand()  # we get the the x,y,parent list
            pygame.draw.circle(map.map, map.grey, (x[-1], y[-1]), map.noderad + 2, 0) # -1 indicate latest or last entry of list
            pygame.draw.line(map.map, map.Blue, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]), map.edgeThickness)
            pygame.display.update()
        end_time = time.time()
        self.time = end_time - start_time
        print("Time Needed = " + str(self.time))
            

        path = graph.getPathCoords()
        print(path)
        for i in range(len(path)):
            if(i+1 < len(path)):
                self.distance  =  self.distance + math.hypot(abs(path[i][0]-path[i+1][0]),abs(path[i][1]-path[i+1][1]))

        print(self.distance)
        map.drawpath(path)
        pygame.display.update()
        
        while True:
            pygame.display.update()
            for e in pygame.event.get():
                if e.type==QUIT:
                    pygame.quit()
                    sys.exit()

        #pygame.event.clear()
        #pygame.event.wait(0)



