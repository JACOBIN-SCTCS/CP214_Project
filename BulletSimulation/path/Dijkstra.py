

import os
import sys
import math
import heapq
import pickle
import plotting, env
from Astar import AStar
import parameters


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


def main():
    s_start = parameters.START_POS
    s_goal = parameters.END_POS

    dijkstra = Dijkstra(s_start, s_goal, 'None')
    plot = plotting.Plotting(s_start, s_goal,dijkstra.Env)

    parameter = open("parameters",'wb')
    path, visited = dijkstra.searching()

    params = dict()
    params["start_pos"] = s_start
    params["end_pos"] = s_goal
    params["obstacles"] = dijkstra.Env.obs
    path.reverse()
    params["path"] = path
    print(path)
    pickle.dump(params,parameter)
    parameter.close()

    plot.animation(path, visited, "Dijkstra's")  # animation generate


if __name__ == '__main__':
    main()
