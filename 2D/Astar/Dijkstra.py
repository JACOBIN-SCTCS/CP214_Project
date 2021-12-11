import os
import sys
import math
import heapq
import Astar.plotting as plotting, Astar.env as env
from Astar.Astar import AStar

class Dijkstra(AStar):
    def searching(self):


        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (0, self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:
                break

            for s_n in self.get_neighbor(s):



                if(s_n[0]<0 or s_n[0]>= self.Env.x_range or s_n[1] < 0 or s_n[1]>= self.Env.y_range):
                    continue

                #print(s_n)
                collided = False
                for obstacles in self.Env.obs:
                    if(obstacles[0] == s_n[0] and obstacles[1] == s_n[1]):
                        collided = True
                        break
                        
                if(collided):
                    continue

                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s

                    # best first set the heuristics as the priority 
                    heapq.heappush(self.OPEN, (new_cost, s_n))

        return self.extract_path(self.PARENT), self.CLOSED

