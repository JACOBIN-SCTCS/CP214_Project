
import numpy as np
import parameters

class Env:
    def __init__(self,start_node,end_node):
        self.x_range = parameters.GRID_SIZE[0]  # size of background
        self.y_range = parameters.GRID_SIZE[1]
        
        self.start = start_node
        self.end = end_node

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):

        x = self.x_range
        y = self.y_range
        obs = set()
        
        n = 100
        x_coord = np.random.randint(0,x,n).tolist()
        y_coord = np.random.randint(0,y,n).tolist()

        for i in range(n):
            if( (x_coord[i] == self.start[0] and y_coord[i] == self.start[1]) 
              or (x_coord[i] == self.end[0] and y_coord[i] == self.end[1])):
                continue

            obs.add((x_coord[i],y_coord[i]))


        return obs 

