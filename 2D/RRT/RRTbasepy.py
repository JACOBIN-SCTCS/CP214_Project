import random
import math
import pygame
class RRTMap:
    def __init__(self,start,goal,MapDimensions,obsdim,obsnum):
        self.start=start
        self.goal=goal
        self.MapDimensions=MapDimensions
        self.Maph,self.MapW=self.MapDimensions

        # window setting
        self.MapwindoneName='RRT path plannning'
        pygame.display.set_caption(self.MapwindoneName)
        self.map=pygame.display.set_mode((self.MapW,self.Maph)) #create a map
        self.map.fill((255,255,255))
        self.noderad=2
        self.nodeThickness=0
        self.edgeThickness=1

        self.obstacles=[]
        self.obsdim=obsdim
        self.obsNumber=obsnum

        # color
        self.grey =(70,70,70)
        self.Blue = (0,0,255)
        self.Green=(0,255,0)
        self.Red = (255,0,0)
        self.white=(255,255,255)
    def drawMap(self,obstacles): # draw start and end point as circle and draw obs by calling drawObs function
        pygame.draw.circle(self.map,self.Green,self.start,self.noderad+5,0)
        pygame.draw.circle(self.map, self.Green, self.goal, self.noderad + 20, 1)
        self.drawObs(obstacles)
    def drawpath(self,path):
        for node in path:
            pygame.draw.circle(self.map,self.Red,node,self.noderad+3,0)

    def  drawObs(self,obstacles): # draw obs
        obstaclesList=obstacles.copy()
        while(len(obstaclesList)>0):
            obstacles=obstaclesList.pop(0)
            pygame.draw.rect(self.map,self.grey,obstacles)

class RRTGraph:
    def __init__(self,start,goal,MapDimensions,obsdim,obsnum):
        (x,y)=start
        self.start=start
        self.goal=goal
        self.goalFlag=False
        self.maph,self.mapw=MapDimensions
        self.x=[]
        self.y=[]
        self.parent=[]
        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        # the obstacles
        self.obstacles=[]
        self.obsDim=obsdim
        self.obsNum=obsnum
        # path
        self.goalstate=None
        self.path=[]
    def makeRandomRect(self):   # Randomly generates upper corner coordinates within the environment
        uppercornerx=int(random.uniform(0,self.mapw-self.obsDim))
        uppercornery=int(random.uniform(0,self.mapw-self.obsDim))
        return (uppercornerx,uppercornery)
    
    def makeobs(self,obstacles): # remove all the obstacles which colloide with the start and end point and add remaining to obs list
        obs=[]
        for rect in obstacles:
            rectange=None
    

            rectange= pygame.Rect(rect[0],rect[1],self.obsDim,self.obsDim)

            if rectange.collidepoint(self.start) or rectange.collidepoint(self.goal):
                    continue
            obs.append(rectange)
        self.obstacles=obs.copy()
        return  obs

    def add_node(self,n,x,y):
        self.x.insert(n,x)
        self.y.append(y)
    def remove_node(self,n):
        self.x.pop(n)
        self.y.pop(n)
    def add_edge(self,parent,child):
        self.parent.insert(child,parent)

    def remove_edge(self,n):
        self.parent.pop(n)
    def number_of_nodes(self):
        return len(self.x)
    def distance(self,n1,n2):
        (x1,y1)=(self.x[n1],self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px=(float(x1)-float(x2))**2
        py=(float(y1)-float(y2))**2
        return (px+py)**0.5
    def sample_envir(self): #generates random sample nodes
        x=int(random.uniform(0,self.mapw))
        y=int(random.uniform(0,self.maph))
        return x,y
    def nearest(self,n):
        dmin=self.distance(0,n)  # start with the distance bet start node and newly add node
        nnear=0 #initial index of nearest node is 0 i.e. the start node
        for i in range(0,n):
            if self.distance(i,n)<dmin:  #if the distance between ith node and newly added node is less the dmin then we re-asign dmin
                dmin=self.distance(i,n)
                nnear=i #now the node with ith index is the nearest node
        return nnear
    def isFree(self):
        n=self.number_of_nodes()-1
        (x,y)=(self.x[n],self.y[n])
        obs=self.obstacles.copy()
        while len(obs)>0:
            rectange=obs.pop(0)
            if rectange.collidepoint(x,y):
                self.remove_node(n)
                return False
        return  True
    def crossObstacles(self,x1,x2,y1,y2):
        obs=self.obstacles.copy()
        while(len(obs)>0):
            rectange=obs.pop(0)
            for i in range(0,101):
                u=i/100
                x=x1*u + x2*(1-u)
                y = y1 * u + y2 * (1 - u)
                if rectange.collidepoint(x,y):
                    return True
        return False
    def connect(self,n1,n2):
        (x1,y1)=(self.x[n1],self.y[n1])
        (x2,y2)=(self.x[n2],self.y[n2])
        if self.crossObstacles(x1,x2,y1,y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True
    def step(self,nnear,nrand,dmax=35):  #max distance of edge is 35
        d=self.distance(nnear,nrand)
        if d>dmax:
            u=dmax/d
            (xnear,ynear)=(self.x[nnear],self.y[nnear])
            (xrand,yrand)=(self.x[nrand],self.y[nrand])
            (px,py)=(xrand-xnear,yrand-ynear)
            theta=math.atan2(py,px)
            (x,y)=(int(xnear+dmax * math.cos(theta)),int(ynear+dmax * math.sin(theta))) # new node which is at a distance 35 from the nnear
            self.remove_node(nrand)
            if abs(x-self.goal[0])<dmax and abs(y-self.goal[1])<dmax:
                self.add_node(nrand,self.goal[0],self.goal[1])
                self.goalstate=nrand
                self.goalFlag=True
            else:
                self.add_node(nrand,x,y)

    def expand(self):
        n=self.number_of_nodes()  # index of parent node
        x,y=self.sample_envir()  # a new node is samples
        self.add_node(n,x,y) #added to the tree temporarily
        if self.isFree(): # if the temporary node is free node
            xnearest=self.nearest(n) # find the nearest node to the temporary node
            self.step(xnearest,n)
            self.connect(xnearest,n)
        return self.x,self.y,self.parent
    
    def path_to_goal(self): # get the index of all the parent node
        if self.goalFlag:
            self.path=[]
            self.path.append(self.goalstate)
            newpos=self.parent[self.goalstate]
            while newpos!=0:
                self.path.append(newpos)
                newpos=self.parent[newpos]
            self.path.append(0)
        return self.goalFlag
    
    def getPathCoords(self): # get the coordinate of parent node
        pathCoords=[]
        for node in self.path:
            x,y=(self.x[node],self.y[node])
            pathCoords.append((x,y))
        return pathCoords

