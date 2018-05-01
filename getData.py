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

from sensor_msgs.msg import Image

VERBOSE=False

class image_feature:

    def __init__(self, folder_name):
        self.folder_name = folder_name
        self.subscriber = rospy.Subscriber("/image",
                                           Image, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /image"

    def callback(self, ros_data):
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
