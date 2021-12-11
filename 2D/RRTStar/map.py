import time
import pygame
from pygame.locals import *
from parameters import *
import math
import random
import sys
import numpy as np

class RRTStar:
    def __init__(self,W,H,screen,numObstacles = 100):
        self.obstacles = []
        self.nObstacles = numObstacles
        self.obstacle_dim = OBSTACLE_DIM
        self.delta = DELTA
        self.window_width = W
        self.window_height = H
        self.starting_point = ()
        self.ending_point = ()
        self.num_iterations  = NUM_ITERATIONS
        self.steer_distance = STEER_DISTANCE
        self.rewire_radius = REWIRE_RADIUS
        self.screen =screen
        self.reached_goal = False
        self.x = []
        self.y = []
        self.distance = [] 
        self.parent = []
        self.dist = 0
        self.time = 0


    def generate_obstacles(self,obstacles):
        '''x_coord = np.random.randint(0,self.window_height-self.obstacle_dim,self.nObstacles).tolist()
        y_coord = np.random.randint(0,self.window_width-self.obstacle_dim,self.nObstacles).tolist()
        
        for i in range(len(x_coord)):
            obstacle = Rect(x_coord[i],y_coord[i],self.obstacle_dim,self.obstacle_dim)
            self.obstacles.append(obstacle)
            '''

        for i in obstacles:
            obstacle = Rect(i[0],i[1],self.obstacle_dim,self.obstacle_dim)
            self.obstacles.append(obstacle)


    def get_nearest_node_idx(self,x_new,y_new):
        distance = math.hypot(self.x[0]-x_new,self.y[0]-y_new)
        nearest_node_idx = 0
        for i in range(len(self.x)):
            cur_distance = math.hypot(self.x[i]-x_new,self.y[i]-y_new)
            if (cur_distance< distance):
                distance = cur_distance
                nearest_node_idx = i
        return nearest_node_idx

    def rewire(self,current_idx,parent_idx):
       
       for i in range(len(self.x)):
           if(i != current_idx and i!=parent_idx):
               
               x_coord,y_coord = self.x[i],self.y[i]
               dist = math.hypot(x_coord-self.x[current_idx],y_coord-self.y[current_idx])
               if(dist<= self.rewire_radius):

                   neighbouring_distance = 0.0
                   curr = prev = i 
                   while(self.parent[curr]!=-1):
                       prev = curr 
                       curr = self.parent[curr]
                       neighbouring_distance += math.hypot(self.x[prev]-self.x[curr],self.y[prev]-self.y[curr])
                   if(neighbouring_distance >  self.distance[current_idx]+dist):
                       
                       obstacles_present = False
                       for k in range(200):
                           u = k/200
                           x_line = int((u*self.x[i])+ ((1-u)*self.x[current_idx]))
                           y_line = int((u*self.y[i])+ ((1-u)*self.y[current_idx]))

                           goal = self.ending_point 
                           if((goal[0]-self.delta<=x_line <=goal[0]+self.delta) and ((goal[1]-self.delta<=y_line <=goal[1]+self.delta))):
                               self.reached_goal = True
                               end_x = self.ending_point[0]
                               end_y = self.ending_point[1]
                               self.x.append(end_x)
                               self.y.append(end_y)
                               self.parent.append(i)
                               self.distance.append(neighbouring_distance+math.hypot(self.x[i]-end_x,self.y[i]-end_y))
                               return 
                           for obstacle in self.obstacles:
                               if (obstacle.collidepoint((x_line,y_line))):
                                   obstacles_present = True
                                   break
                           if(obstacles_present):
                               break
                       
                       if(not obstacles_present):
                           pygame.draw.line(self.screen,WHITE_COLOR,(self.x[self.parent[i]],self.y[self.parent[i]]),(self.x[i],self.y[i])) 
                           self.parent[i] = current_idx
                           pygame.draw.line(self.screen,BLACK_COLOR,(self.x[i],self.y[i]),(self.x[current_idx],self.y[current_idx]))
                           self.distance[i] = self.distance[current_idx] + dist
                        


    
    def plot_path(self):
        current_idx = len(self.x)-1
        prev = current_idx
        self.dist = 0
        while(self.parent[current_idx]!=-1):
            prev = current_idx
            current_idx = self.parent[current_idx]
            pygame.draw.line(self.screen,RED_COLOR,(self.x[current_idx],self.y[current_idx]),(self.x[prev],self.y[prev]),3)
            self.dist += math.hypot(abs(self.x[current_idx]-self.x[prev]),abs(self.y[current_idx]-self.y[prev]))
        
        print("Distance" + str(self.dist))


    def start_planning(self):
        self.x.append(self.starting_point[0])
        self.y.append(self.starting_point[1])
        self.distance.append(0.0)
        self.parent.append(-1)

        self.reached_goal = False
        t=0
        while(t<self.num_iterations):
            x_rand = np.random.randint(0,self.window_height)
            y_rand = np.random.randint(0,self.window_width)
            
            idx = self.get_nearest_node_idx(x_rand,y_rand)
            
            #candidate_node=[x_rand,y_rand]
            dist = 0.0
            dist = math.hypot(x_rand-self.x[idx],y_rand-self.y[idx])
            x_steer = 0
            y_steer = 0

            if(dist<=self.steer_distance):
                x_steer = x_rand
                y_steer = y_rand
            else:
                angle = math.atan2(y_rand-self.y[idx],x_rand-self.x[idx])
                x_steer = int(self.x[idx] + self.steer_distance*math.cos(angle))
                y_steer = int(self.y[idx]+ self.steer_distance*math.sin(angle))
                dist = self.steer_distance 

            obstacle_collide = False
            for obstacle in self.obstacles:
                if(obstacle.collidepoint((x_steer,y_steer))):
                    obstacle_collide = True
                    break
            if(obstacle_collide):
                t+=1
                continue
            
            obstacle_edge = False
            for i in range(0,200):
                u = i/200  
                x_line = int((u*self.x[idx])+((1-u)*x_steer))
                y_line = int((u*self.y[idx])+((1-u)*y_steer))
                
                goal = self.ending_point
                if((goal[0]-self.delta<=x_line <=goal[0]+self.delta)
                    and ((goal[1]-self.delta<=y_line <=goal[1]+self.delta))):

                    self.reached_goal= True
                    x_steer = goal[0]
                    y_steer = goal[1]
                    dist = math.hypot(x_steer-self.x[idx],y_steer-self.y[idx])

                    break 
                
                for obstacle in self.obstacles:
                    if(obstacle.collidepoint((x_line,y_line))):
                        obstacle_edge = True
                        break

                if(obstacle_edge or self.reached_goal):
                    break
            
            if(not obstacle_edge ):
                self.x.append(x_steer)
                self.y.append(y_steer)
                self.parent.append(idx)

                parent_distance = 0.0 
                prev = current_node = idx
                while(self.parent[current_node]!=-1):
                    prev = current_node
                    current_node = self.parent[current_node]
                    parent_distance +=  math.hypot(self.x[prev]-self.x[current_node],self.y[prev]-self.y[current_node])
                
                self.distance[idx] = parent_distance

                self.distance.append(self.distance[idx]+ dist)
                #print(str(self.x[-1])+","+str(self.y[-1])+"d:"+str(self.distance[-1]))
                pygame.draw.circle(self.screen,BLACK_COLOR,(x_steer,y_steer),2)
                pygame.draw.line(self.screen,BLACK_COLOR,(self.x[idx],self.y[idx]),(x_steer,y_steer))
                #myfont = pygame.font.SysFont('Comic Sans MS', 15)
                #textsurface = myfont.render(str(int(self.distance[-1])), False, (0, 0, 0))
                #self.screen.blit(textsurface,(x_steer,y_steer))
                pygame.display.update()
                self.rewire(len(self.x)-1,idx)

            if(self.reached_goal):
                print("Destination reached")
                break
            t+=1
        if(self.reached_goal):
            self.plot_path()


    def set_start_end_point(self,start,end):
        for obstacle in self.obstacles:
            if obstacle.collidepoint(start) or obstacle.collidepoint(end):
                print("Collision Detected")
                pygame.quit()
                sys.exit()
        
        pygame.draw.circle(self.screen,GREEN_COLOR,start,7)
        pygame.draw.circle(self.screen,RED_COLOR,end,7)

        self.starting_point = start
        self.ending_point = end
        start_time = time.time()
        self.start_planning()
        end_time = time.time()
        self.time = end_time - start_time
        print("Time Needed" + str(self.time))
    

class Map:

    def __init__(self,wH=WINDOW_HEIGHT,wW=WINDOW_WIDTH):
        
        self.screen = pygame.display.set_mode((wH,wW))
        self.screen.fill(WHITE_COLOR)
        self.obstacle_color = (128,128,128)
        self.starting_point_color = RED_COLOR
        self.ending_point_color = GREEN_COLOR
        self.rrt_star  = RRTStar(wW,wH,self.screen)

    def set_starting_ending_point(self,start,end):
        self.rrt_star.set_start_end_point(start,end) 

    def drawObstacles(self,obstacles):
        self.rrt_star.generate_obstacles(obstacles)
        for obstacle in self.rrt_star.obstacles:
            pygame.draw.rect(self.screen,self.obstacle_color,obstacle)
        




