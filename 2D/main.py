
import numpy as np
from RRT.RRT import RRTMain
from RRTStar.RRTStar import RRTStarMain
from RRTStar.map import RRTStar
import Astar.plotting as plotting
from Astar.Astar import AStar
from Astar.Dijkstra import Dijkstra
import parameters
import time
import pickle
import os 

rrt_star = False

start_pos = (2,5)
end_pos = (40,30)
n_obstacles = 50
window_width = 150
window_height = 150

obstacle_file_name = 'obstacle_file'
rrt_window_width = 650
rrt_window_height = 650
rrt_start_pos = (10,25)
rrt_end_pos = (590,560)
rrt_n_obstacles = 200
obstacle_dim = parameters.OBSTACLE_DIM


def createObstacles():
    obs = set()
    n = 50
    x_coord = np.random.randint(0,window_width,n).tolist()
    y_coord = np.random.randint(0,window_height,n).tolist()

    for i in range(n):
        if( (x_coord[i] == start_pos[0] and y_coord[i] == start_pos[1]) 
              or (x_coord[i] == end_pos[0] and y_coord[i] == end_pos[1])):
            continue

        obs.add((x_coord[i],y_coord[i]))


    return obs 

def createObstacles_RandomSampling():
    
    obs = set()
    x_coord = np.random.randint(0,rrt_window_width-obstacle_dim,rrt_n_obstacles).tolist()
    y_coord = np.random.randint(0,rrt_window_height-obstacle_dim,rrt_n_obstacles).tolist()

    file = open(obstacle_file_name,'ab')   
    for i in range(len(x_coord)):
        if(
            (x_coord[i]>= rrt_start_pos[0] and x_coord[i] <= rrt_start_pos[0] + obstacle_dim) or
            (y_coord[i]>= rrt_start_pos[1] and y_coord[i] <= rrt_start_pos[1] + obstacle_dim) or
            (x_coord[i]>= rrt_end_pos[0] and x_coord[i] <= rrt_end_pos[0] + obstacle_dim) or
            (y_coord[i]>= rrt_end_pos[1] and y_coord[i] <= rrt_end_pos[1] + obstacle_dim) 
            ):
            continue

        obs.add((x_coord[i],y_coord[i]))
    pickle.dump(obs,file)
    file.close()
    return obs





def main():

    obstacles = createObstacles()
    
    '''astar = AStar(start_pos, end_pos, obstacles, "euclidean")
   
    # call time 1231312313
    
    plot = plotting.Plotting(start_pos, end_pos,astar.Env)
    
    time_start = time.time()
    path, visited = astar.searching()
    time_end = time.time()
    print(time_end-time_start)
    
    print(len(path))
    plot.animation(path, visited, "A*",time_end-time_start ) 
 
    '''
    dijkstra = Dijkstra(start_pos,end_pos,obstacles,None)
    plot = plotting.Plotting(start_pos, end_pos,dijkstra.Env)
    
    time_start = time.time()
    path, visited = dijkstra.searching()
    time_end = time.time()
    print(time_end-time_start)


    print(len(path))
    plot.animation(path, visited, "Dijkstra",time_end-time_start) 
    
    '''

    rrt_obstacles = None
    if os.path.exists(obstacle_file_name):
        picklefile = open(obstacle_file_name, 'rb')     
        rrt_obstacles = pickle.load(picklefile)
        picklefile.close()

    else:
        rrt_obstacles = createObstacles_RandomSampling()


    
    if rrt_star:
        rrtstar = RRTStarMain(rrt_start_pos,rrt_end_pos,rrt_obstacles,obstacle_dim,rrt_window_width,rrt_window_height)
        rrtstar.run()
    else:
        rrt = RRTMain(rrt_window_width,rrt_window_height,rrt_start_pos,rrt_end_pos,rrt_obstacles,len(rrt_obstacles))
        rrt.run()
    '''

if __name__ == "__main__":
    main()
