import sys, time
import os
import datetime
import numpy as np
from PIL import Image as I
from PIL import Image
from scipy.ndimage import filters
import cv2, array
import roslib
import rospy
import detect_blue
from std_msgs.msg import Float32MultiArray

#detect_blue.main('1525142837.59.png')

from sensor_msgs.msg import Image

VERBOSE=False

transformed_points = []
# Focal lengths of Camera
fx = 0.45
fy = 0.24

# Camera's position in the world frame (Assuming there's no rotation)
Xw_c = 2.85
Yw_c = -4.65
Zw_c = 0.0

class image_feature:

    def __init__(self, folder_name):
        self.folder_name = folder_name
        self.subscriber = rospy.Subscriber("/image",
                                           Image, self.callback,  queue_size = 1)
        #self.publisher = rospy.Publisher('/coordinates', Float64, queue_size=1)
        if VERBOSE :
            print "subscribed to /image"

    def Transform_C_W(self,xi, yi, Xw_c, Yw_c, Zw_c, fx, fy ):
        xf = (-320 + xi)*0.001
        yf = (240 - yi)*0.001
        Xc_o = xf*(-Yw_c)/fx
        Yc_o = yf*(-Yw_c)/fy
        Zc_o = -Yw_c
        print(Xc_o)
        print(Xw_c)
        Xw_o = Xw_c + Xc_o
        Yw_o = Zc_o + Yw_c
        Zw_o = Zw_c + Yc_o
        return Xw_o, Zw_o

    def callback(self, ros_data):
        msg = Float32MultiArray()
        #pub_coordinates = rospy.Publisher('/coordinates', Float64, queue_size=1)
        coord_points = []
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        from PIL import Image
        np_arr = ros_data.data
        image_byte_array = array.array('b', np_arr)
        image_buffer = I.frombuffer("RGB", (640,480), image_byte_array, "raw", "BGR", 0, 1)
        image_buffer  = image_buffer.transpose(Image.FLIP_LEFT_RIGHT)
        image_buffer  = image_buffer.transpose(Image.ROTATE_180)
        img2 = np.asarray(image_buffer)
        time2 = time.time()
        center_coord = detect_blue.main(img2)
        if center_coord is not None:
            center_trans = self.Transform_C_W(center_coord[0], center_coord[1], Xw_c, Yw_c, Zw_c, fx, fy )
            transformed_points.append(center_trans)
            if(len(transformed_points)>2):
                #break
                msg.data = [transformed_points[0][0], transformed_points[0][1], transformed_points[1][0], transformed_points[1][1], transformed_points[2][0], transformed_points[2][1]]
                pub_coords = rospy.Publisher('/coordinates', Float32MultiArray, queue_size=1)
                pub_coords.publish(msg)
                #print(transformed_points)


        if VERBOSE :
            print '%s detector found: %s points in: %s sec.'%(method,
                                                              len(featPoints),time2-time1)
        if len(str(time2)) > 12:
            cv2.imwrite(self.folder_name + "/" + str(time2)+".png", img2)


def main():
    print('in getdata script')
    a = datetime.datetime.now()
    folder_name = "/home/arpit/502 project/v rep/rgb/" +str(a.day)+ '_' + str(a.hour)+ '_' + str(a.minute) + '_' + str(a.second)
    print(folder_name)
    os.makedirs(folder_name)
    '''Initializes and cleanup ros node'''
    ic = image_feature(folder_name)
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
