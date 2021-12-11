import pybullet as p
import time
import pybullet_data
import math
import pickle


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

dbfile = open('parameters','rb')
params = pickle.load(dbfile)
dbfile.close()

start_pos = params["start_pos"]
end_pos = params["end_pos"]
obstacles = list(params["obstacles"])
positions  = params["path"]

planeId = p.loadURDF("plane.urdf")
startPos = [start_pos[0],start_pos[1],.2]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("urdf/robot2.urdf",startPos, startOrientation)

print(p.getNumJoints(boxId))

for i in range(p.getNumJoints(boxId)):
    print(p.getJointInfo(boxId,i))

for i in range(len(obstacles)):
    cubeid = p.loadURDF("urdf/box.urdf",[obstacles[i][0],obstacles[i][1],0.145],p.getQuaternionFromEuler([0,0,0]),useFixedBase=1)

cubeid = p.loadURDF("urdf/dest.urdf",[positions[-1][0],positions[-1][1],-0.145],p.getQuaternionFromEuler([0,0,0]) , useFixedBase=1)
#cubeid = p.loadURDF("urdf/dest.urdf",[0,0,-0.145],p.getQuaternionFromEuler([0,0,0]) , useFixedBase=1)

maxforce = 30

i=0
while True:

    while i< len(positions):
        print(str(i)+ "th point under process")
        next_point = positions[i]
        position = p.getBasePositionAndOrientation(boxId) 
        x, y = position[0][0], position[0][1]
    
        dd = 0.2
        if(x>=next_point[0]-dd and x<= next_point[0]+dd  and y>=next_point[1]-dd and  y<=next_point[1]+dd):
            p.setJointMotorControl2(bodyUniqueId=boxId, 
                jointIndex=1, 
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity = 0,
                force = maxforce)

            p.setJointMotorControl2(bodyUniqueId=boxId, 
                jointIndex=2, 
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity = 0,
                force = maxforce)
            
            
            p.stepSimulation()
            time.sleep(1./240)
            i+=1
            if(i==len(positions)):
                while(True):

                    p.setJointMotorControl2(bodyUniqueId=boxId, 
                    jointIndex=1, 
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity = -0.1,
                    force = maxforce)

                    p.setJointMotorControl2(bodyUniqueId=boxId, 
                        jointIndex=2, 
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity = 0.1,
                        force = maxforce)
                    p.stepSimulation()
                    time.sleep(1./240)

            
            continue
            
        theta = p.getEulerFromQuaternion(position[1])[2]

        inc_x = next_point[0] - x 
        inc_y = next_point[1] - y
        angle_to_goal = math.atan2(inc_y,inc_x)

        linear_velcity = 0
        angular_velocity = 0


        if(abs(angle_to_goal - theta) > 0.2):
            linear_velcity = 0.0
            angular_velocity = 0.7
        else:
            linear_velcity = 0.7
            angular_velocity = 0.0

        right_velocity = (2*linear_velcity - angular_velocity*0.3)/(2*0.04)
        left_velocity = (2*linear_velcity + angular_velocity*0.3)/(2*0.04)

        p.setJointMotorControl2(bodyUniqueId=boxId, 
                jointIndex=1, 
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity = right_velocity,
                force = maxforce)

        p.setJointMotorControl2(bodyUniqueId=boxId, 
                jointIndex=2, 
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity = left_velocity,
                force = maxforce)

        p.stepSimulation()
        time.sleep(1./240)
    break
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

    

