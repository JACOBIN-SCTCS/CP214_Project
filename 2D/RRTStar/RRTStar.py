import pygame 
from pygame.locals import *
import sys
from RRTStar.map import Map


class RRTStarMain:

    def __init__(self,start_point,end_point,obstacles,obstacle_dim,window_width, window_height):
        self.starting_point = start_point
        self.ending_point = end_point
        self.obs = obstacles
        self.obs_dim = obstacle_dim
        self.window_width = window_width
        self.window_height = window_height

    def run(self):
        pygame.init()
        pygame.font.init()

        m = Map(self.window_width,self.window_height)
        m.drawObstacles(self.obs)

        m.set_starting_ending_point(self.starting_point,self.ending_point)

        while True:
            pygame.display.update()
            for e in pygame.event.get():
                if e.type==QUIT:
                    pygame.quit()
                    sys.exit()
