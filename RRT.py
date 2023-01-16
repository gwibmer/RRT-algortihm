import sys
import numpy as np
sys.path.append('PythonAPI')
import math
import time
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit('Could not connect to Vrep')
# get the handles of arm joints
err_code, armjoint1_handle = sim.simxGetObjectHandle(clientID,"UR5_joint1", 
sim.simx_opmode_blocking)
err_code, armjoint2_handle = sim.simxGetObjectHandle(clientID,"UR5_joint2", 
sim.simx_opmode_blocking)
err_code, armjoint3_handle = sim.simxGetObjectHandle(clientID,"UR5_joint3", 
sim.simx_opmode_blocking)
err_code, armjoint4_handle = sim.simxGetObjectHandle(clientID,"UR5_joint4", 
sim.simx_opmode_blocking)
err_code, armjoint5_handle = sim.simxGetObjectHandle(clientID,"UR5_joint5", 
sim.simx_opmode_blocking)
err_code, armjoint6_handle = sim.simxGetObjectHandle(clientID,"UR5_joint6", 
sim.simx_opmode_blocking)
# get the handles of hand joints
err_code, endeffector_handle = sim.simxGetObjectHandle(clientID,"suctionPad", 
sim.simx_opmode_blocking)
# set the arm to position control
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2001, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2000, 1, 
sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2001, 1, 
sim.simx_opmode_oneshot)
# get the collision handles
collision_handle_list = []
for i in range(40):
    err_code, collision_handle = sim.simxGetCollisionHandle(clientID, "Collision" +
str(i), sim.simx_opmode_blocking)
    sim.simxReadCollision(clientID, collision_handle, sim.simx_opmode_streaming)
    collision_handle_list.append(collision_handle)
# You do not need to modify the code above
# function to control the movement of the arm, the input are the angles of joint1, 

def move_arm(armpose):
    armpose_convert = []
    for i in range(6):
        armpose_convert.append(round(armpose[i]/180 * math.pi,3))
    sim.simxPauseCommunication(clientID,True)
    sim.simxSetJointTargetPosition(clientID, armjoint1_handle, armpose_convert[0], 
sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint2_handle, armpose_convert[1], 
sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint3_handle, armpose_convert[2], 
sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint4_handle, armpose_convert[3], 
sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, armjoint5_handle, armpose_convert[4], 
sim.simx_opmode_oneshot) 
    sim.simxSetJointTargetPosition(clientID, armjoint6_handle, armpose_convert[5], 
sim.simx_opmode_oneshot)    
    sim.simxPauseCommunication(clientID,False)
    time.sleep(3)
# function to check collision
def check_collision():
    collision_reading = np.zeros(40)
    is_collision = 0
    for i in range(40):
        collision_reading[i] = sim.simxReadCollision(clientID, 
collision_handle_list[i], sim.simx_opmode_buffer)[1]
        if collision_reading[i] == 1:
            is_collision = 1
    if is_collision == 1:
        print('Collision detected!')
        return 1
    else:
        return 0





#purpose:Helper function to calulast distance for nearest
#inputs: actual pose and the gooal pose
#assumption: atucal its a set of 6 angles
#post condition:the desitance will be returned
def distance(actual,goal):
    actual_distance = (goal[0]-actual[0]) + (goal[1]-actual[1]) + (goal[2]-actual[2]) + (goal[3]-actual[3]) + (goal[4]-actual[4]) + (goal[5]-actual[5]) 
    return actual_distance
    


#purpose: RRT motion planning algorithm
#inputs:actual pose and the gooal pose
#assumption:atucal its a set of 6 angles
#post condition: The desire path will be returned
def RRT(start,goal):
    move_arm(start)
    print(start)
    #creating a pose in case of collsion
    old_pose = start
    # Create instances to save the nearest , the path list and free poses list
    free_poses =[]
    nearest =[]
    path = []
     # we will loop until we get to our goal
    while nearest != goal:
        # getting distance from the start to the goal
        before = distance(start,goal)
        #append the start to the path
        path.append(start)
        #getting 50 random  poses to decide which one is the nearest
        for i in range(20):
            # getting the restircted random poses
            rand_target_arm= [np.random.randint(start[0], goal[0]+1),np.random.randint(start[1], goal[1]+1),np.random.randint(start[2], goal[2]+1), np.random.randint(start[3], goal[3]+1),np.random.randint(start[4], goal[4]+1), np.random.randint(start[5], goal[5]+1)]
            # appending those poses to the free poses list
            free_poses.append(rand_target_arm)
        # going through the lsit of poses
        for i in range(len(free_poses)):
            # gettingthe distance from the actual pose of the list
            current = distance(free_poses[i],goal)
            #comparing it with the distance from the before pose
            if current < before:
                # if it is near to the goal we save it in the nearest varaible
                nearest = free_poses[i]
                #and we make our actual to be the before
                before = current
        # checking for collsion
        if check_collision() !=0:
            # if there is a collsion the arm will move to the old pose
            move_arm(old_pose)
            print(old_pose)
            start =old_pose
      
        else:
            #append the nearest out of all the poses genrated
            path.append(nearest)
            move_arm(nearest)
            print(nearest)
            #my new start will be the nearest
            old_pose = start
            start = nearest
           
        
    return path
        
    
# test script to test my RRT function
start = [30,30,30,30,30,30]
goal = [80,80,80,80,80,80] 

    
path_constructed = RRT(start,goal)  





# no need to modify the code below
# close the communication between collision handles
for i in range(40):
    sim.simxReadCollision(clientID, collision_handle_list[i], 
sim.simx_opmode_discontinue)
print ('Program ended')