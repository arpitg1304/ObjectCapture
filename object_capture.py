import vrep
import time
import numpy as np
import sys, termios, tty, os, time
import csv
import cv2, array

from PIL import Image as I
from PIL import Image
from multiprocessing import Process

import getData
def getSensorPose(clientID, handle):
    res, floor_handle = vrep.simxGetObjectHandle(clientID, 'Graph', vrep.simx_opmode_blocking)
    res, s_handle = vrep.simxGetObjectHandle(clientID, 'kinect', vrep.simx_opmode_blocking)
    position = vrep.simxGetObjectPosition(clientID, s_handle, handle, vrep.simx_opmode_buffer )
    return [position]

def runInParallel(*fns):
    proc = []
    for fn in fns:
        p = Process(target=fn)
        p.start()
        proc.append(p)
    for p in proc:
        p.join()

def getData():
    img2, img_time = fetchKinect(clientID,'kinect_depth','kinect_rgb')
    cv2.imwrite(str(img_time)+'.png',img2)


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def fetchKinect(clientID,depthSTR,rgbSTR):
    errorCodeKinectRGB,kinectRGB=vrep.simxGetObjectHandle(clientID,rgbSTR,vrep.simx_opmode_blocking)

    #errorCodeKinectDepth,kinectDepth=vrep.simxGetObjectHandle(clientID,depthSTR,vrep.simx_opmode_blocking)

    #errorHere,resol,depth=vrep.simxGetVisionSensorDepthBuffer(clientID,kinectDepth,vrep.simx_opmode_oneshot_wait)
    #depthArr=np.array(depth)

    #fetchKinect('kinect_depth','kinect_rgb')

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

if __name__ == "__main__":

    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    #clientID = 0
    res, floor_handle = vrep.simxGetObjectHandle(clientID, 'floor_handle', vrep.simx_opmode_blocking)
    res, s_handle = vrep.simxGetObjectHandle(clientID, 'kinect', vrep.simx_opmode_blocking)
    position = vrep.simxGetObjectPosition(clientID, s_handle, vrep.sim_handle_parent, vrep.simx_opmode_blocking)
    orientation = vrep.simxGetObjectOrientation(clientID, s_handle, vrep.sim_handle_parent, vrep.simx_opmode_buffer)

    print(position)
    print(orientation)

    while True:
        char = getch()

        if (char == "m"):
            print("Stop!")
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

        if (char == "s"):
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
            exit(0)

        if (char == "p"):
            vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot_wait)

        x_req = 1.35 # to be replaced with robot workspace coordinates

        res1, obj1 = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)

        if (char == "n"):


            pose_list = []
            xx = []
            zz = []

            z_vel, x_vel = input("velocity in Z and X: ")
            time1 = input("Time of flight: ")
            vrep.simxSetObjectFloatParameter(clientID, obj1, vrep.sim_shapefloatparam_init_velocity_z, z_vel, vrep.simx_opmode_blocking)
            vrep.simxSetObjectFloatParameter(clientID, obj1, vrep.sim_shapefloatparam_init_velocity_x, x_vel, vrep.simx_opmode_blocking)
            vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

            t_end = time.time() + time1
            #while time.time() < t_end:
            for i in range(0,10):

                errorCodeKinectRGB,kinectRGB=vrep.simxGetObjectHandle(clientID,'kinect_rgb',vrep.simx_opmode_blocking)
                #errorHere,resolution,image=vrep.simxGetVisionSensorImage(clientID,kinectRGB,0,vrep.simx_opmode_blocking)
                img_time=vrep.simxGetLastCmdTime(clientID)


                #errorCodeKinectDepth,kinectDepth=vrep.simxGetObjectHandle(clientID,depthSTR,vrep.simx_opmode_blocking)
                errorHere,resolution,image=vrep.simxGetVisionSensorImage(clientID,kinectRGB,0,vrep.simx_opmode_oneshot_wait)
                image_byte_array = array.array('b', image)
                image_buffer = I.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "BGR", 0, 1)
                image_buffer  = image_buffer.transpose(Image.FLIP_LEFT_RIGHT)
                image_buffer  = image_buffer.transpose(Image.ROTATE_180)

                img2 = np.asarray(image_buffer)
                img_time=vrep.simxGetLastCmdTime(clientID)
                #print(img_time)

                #cv2.imwrite(str(img_time)+'.png',img2)

                time1=vrep.simxGetLastCmdTime(clientID)

                pose = getPosition(clientID)
                pose_list.append(pose)


                if pose[1][0] >= x_req:
                    vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot_wait)

                if len(pose_list) > 0 & len(pose_list) <4:
                    xyz = pose[1]
                    xx.append(xyz[0])
                    zz.append(xyz[2])


                if len(pose_list) >3:
                    a,b,c = calc_parabola_vertex(xx[0], zz[0], xx[1], zz[1], xx[2], zz[2])
                    z_req= (a*(x_req**2))+(b*x_req)+c # 2D parabolic equation : z = a*x^2 + b* x + c
                    if z_req < 0 : z_req = 0
                    print(z_req)

            with open('poses.csv','wb') as out:
                csv_out=csv.writer(out)
                csv_out.writerow(['status','XYZ'])
                for row in pose_list:
                    csv_out.writerow(row)
