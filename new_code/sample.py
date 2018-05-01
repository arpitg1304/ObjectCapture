import vrep
import time
import numpy as fnp
import sys, termios, tty, os, time
import roslib
import rospy

from std_msgs.msg import Float32MultiArray
from nodes import Node
import cv2, array
import numpy as np

#

from PIL import Image as I
from PIL import Image
from multiprocessing import Process
from abb import ABB_IRB_140
robot = ABB_IRB_140()
import matplotlib.pyplot as plt
import getData
def getSensorPose(clientID, handle):
    res, floor_handle = vrep.simxGetObjectHandle(clientID, 'Graph', vrep.simx_opmode_blocking)
    res, s_handle = vrep.simxGetObjectHandle(clientID, 'kinect', vrep.simx_opmode_blocking)
    position = vrep.simxGetObjectPosition(clientID, s_handle, handle, vrep.simx_opmode_buffer )
    return [position]

def callback(data):
    print('inside callback')
    data = data.data
    #print(data)
    #a,b,c = calc_parabola_vertex(data[0], data[1], data[2], data[3], data[4], data[5])
    a,b = calc_straight_line(data[0], data[1], data[4], data[5])
    print([a,b])
    z_req_vision= ((a*(x_req))+(b))/2
    # z_req_vision= (a*(x_req**2))+(b*x_req)+c
    print('z required from vision'+str(z_req_vision))
    print('x required from vision'+str(x_req))
    print('-'*8+'Checking if ball can be blocked'+'-'*8)
    transformation = np.array([[1,0,0,x_req*1000],[0,1,0,0],[0,0,1,z_req_vision*1000],[0,0,0,1]])
    desired_angles = robot.inverseKinematics(transformation)
    if desired_angles:
        print('-'*8+'Moving'+'-'*8)
        error = desired_angles-angles
        #error = np.array(error)
        error_collect.append(error)
        #error_collect.append(error.reshape(3,1))
        sum = np.sum(np.abs(error))
        Kp = [1000*(sum-np.abs(i))/sum for i in error]
        Kv = 100
        velocity_error = np.array([0 ,0 ,0])
        desired_velocity = np.copy(velocity_error)
        for i in range(3):
            while abs(error[i])>epsilon:
                vrep.simxSetJointTargetVelocity(clientID,joints[i],error[i]*9999/abs(error[i]),vrep.simx_opmode_oneshot)
                # if error[i]>0.5:
                #     vrep.simxSetJointForce(clientID,joints[i],abs(Kp*0.5)+(Kv*velocity_error[i]),vrep.simx_opmode_oneshot)
                # else:#+(Kv*velocity_error[i])
                if abs(error[i])<epsilon:
                    vrep.simxSetJointTargetVelocity(clientID,joints[i],0.,vrep.simx_opmode_oneshot)
                    vrep.simxSetJointForce(clientID,joints[i],9999.,vrep.simx_opmode_oneshot)

                vrep.simxSetJointForce(clientID,joints[i],abs(Kp[i]*(error[i]))+Kv*velocity_error[i],vrep.simx_opmode_oneshot)
                error[i] = desired_angles[i] - (vrep.simxGetJointPosition(clientID,joints[i],vrep.simx_opmode_streaming)[1])
                #print(vrep.simxGetJointPosition(clientID,joints[i],vrep.simx_opmode_streaming)[1])

                if abs(error[i])<epsilon:
                    vrep.simxSetJointTargetVelocity(clientID,joints[i],0.,vrep.simx_opmode_oneshot)
                    vrep.simxSetJointForce(clientID,joints[i],9999.,vrep.simx_opmode_oneshot)

                res,velocity = vrep.simxGetObjectFloatParameter(clientID,joints[i],2012,vrep.simx_opmode_blocking)
                velocity_error[i] = desired_velocity[i] - velocity
                #error_collect.append(error)

                #print(error)


        # while np.sum(np.abs(error))>epsilon:
        #     for i in range(6):
        #         if abs(error[i])<epsilon/6.:
        #             vrep.simxSetJointTargetVelocity(clientID,joints[i],0.,vrep.simx_opmode_oneshot)
        #             vrep.simxSetJointForce(clientID,joints[i],9999.,vrep.simx_opmode_oneshot)
        #         else:
        #             vrep.simxSetJointTargetVelocity(clientID,joints[i],error[i]*9999/abs(error[i]),vrep.simx_opmode_oneshot)
        #             vrep.simxSetJointForce(clientID,joints[i],abs(Kp*(error[i])),vrep.simx_opmode_oneshot)
        #             error[i] = desired_angles[i] - (vrep.simxGetJointPosition(clientID,joints[i],vrep.simx_opmode_streaming)[1])
        a = []
        for i in range(3):
            vrep.simxSetJointTargetVelocity(clientID,joints[i],0,vrep.simx_opmode_oneshot)
            vrep.simxSetJointForce(clientID,joints[i],9999,vrep.simx_opmode_oneshot)
            a.append(vrep.simxGetJointPosition(clientID,joints[i],vrep.simx_opmode_streaming)[1])
        # print(a)
    else:
        for i in range(3):
            vrep.simxSetJointTargetVelocity(clientID,joints[i],0.,vrep.simx_opmode_oneshot)
            vrep.simxSetJointForce(clientID,joints[i],9999.,vrep.simx_opmode_oneshot)
    print('hi')
    sub_MTML.unregister()
    # print('Printing whole error collected')
    #print((error_collect))
    #plt.show()

def runInParallel(*fns):
    proc = []
    for fn in fns:
        p = Process(target=fn)
        p.start()
        proc.append(p)
    for p in proc:
        p.join()


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def getPosition(clientID):
    res, obj = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)
    pose = vrep.simxGetObjectPosition(clientID, obj, -1, vrep.simx_opmode_blocking)
    return pose

def calc_parabola_vertex(x1, y1, x2, y2, x3, y3):
    denom = (x1-x2) * (x1-x3) * (x2-x3)
    a = (x3 * (y2-y1) + x2 * (y1-y3) + x1 * (y3-y2)) / denom
    b = (x3*x3 * (y1-y2) + x2*x2 * (y3-y1) + x1*x1 * (y2-y3)) / denom
    c = (x2 * x3 * (x2-x3) * y1+x3 * x1 * (x3-x1) * y2+x1 * x2 * (x1-x2) * y3) / denom
    return a,b,c

def calc_straight_line(x1, z1, x2, z2):
    slope = (z2-z1)/(x2-x1)
    c = z1 - slope*x1
    return slope,c


def transform(pos_wrtworld):
    transmation_matrix = np.array([[-1,0,0,2.23],[0,-1,0,-0.775],[0,0,1,0.0771],[0,0,0,1]])
    pos_wrt_robot = np.linalg.solve(transmation_matrix,np.array(pos_wrtworld))
    return pos_wrt_robot

if __name__ == "__main__":
    rospy.init_node('points_talker', anonymous=True)

    error_collect = []
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    joints = [1]*3
    angles = np.zeros(3)
    for i in range(3):
        errorCode,joints[i] = vrep.simxGetObjectHandle(clientID,'IRB140_joint'+str(i+1),vrep.simx_opmode_blocking)
        angles[i] =(vrep.simxGetJointPosition(clientID,joints[i],vrep.simx_opmode_streaming)[1])
    epsilon =6e-2
    for i in range(3):
        vrep.simxSetJointTargetVelocity(clientID,joints[i],0.,vrep.simx_opmode_oneshot)
        vrep.simxSetJointForce(clientID,joints[i],9999.,vrep.simx_opmode_oneshot)
    # Kp = np.abs(error/np.sum(np.abs(error)))
    #Kp = 2000
    #Kv = 100
    # kp = [2000, 2000,2000]
    # kv = [100,100,100]
    res, floor_handle = vrep.simxGetObjectHandle(clientID, 'floor_handle', vrep.simx_opmode_blocking)
    res, s_handle = vrep.simxGetObjectHandle(clientID, 'kinect', vrep.simx_opmode_blocking)
    position = vrep.simxGetObjectPosition(clientID, s_handle, vrep.sim_handle_parent, vrep.simx_opmode_blocking)
    orientation = vrep.simxGetObjectOrientation(clientID, s_handle, vrep.sim_handle_parent, vrep.simx_opmode_buffer)
    print('-'*8+'Calculating Trajectory'+'-'*8)
    # print(position)
    # print(orientation)
    flag = True
    x_req = 0.30 # to be replaced with robot workspace coordinates
    #
    res1, obj1 = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)

    pose_list = []
    xx = []
    zz = []

    #######################################################Setting Velocity###############################################################################

    z_vel, x_vel, y_vel = 0.8, -5, -0.3
    #z_vel, x_vel, y_vel = 0,0,0
    vrep.simxSetObjectFloatParameter(clientID, obj1, vrep.sim_shapefloatparam_init_velocity_z, z_vel, vrep.simx_opmode_blocking)
    vrep.simxSetObjectFloatParameter(clientID, obj1, vrep.sim_shapefloatparam_init_velocity_x, x_vel, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(clientID, obj1, vrep.sim_shapefloatparam_init_velocity_y, y_vel, vrep.simx_opmode_blocking)
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
    for i in range(0,10):

        time1=vrep.simxGetLastCmdTime(clientID)

        pose = getPosition(clientID)
        pose_list.append(pose)

        if len(pose_list) > 0 & len(pose_list) <4:
            xyz = pose[1]
            xx.append(xyz[0])
            zz.append(xyz[2])


        if len(pose_list) >3:
            a,b,c = calc_parabola_vertex(xx[0], zz[0], xx[1], zz[1], xx[2], zz[2])
            z_req= (a*(x_req**2))+(b*x_req)+c # 2D parabolic equation : z = a*x^2 + b* x + c
            if z_req < 0 : z_req = 0
            print('Position from sim handle'+str(z_req))
            break
    sub_MTML = rospy.Subscriber('/coordinates', Float32MultiArray, callback)
    #if count == 1:
    rospy.spin()




###################################### starting controller code here ######################################
    #Collecting error values for plotting#
