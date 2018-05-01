# Python libs
import sys, time
import os

import datetime

# numpy and scipy
import numpy as np
from PIL import Image as I
from PIL import Image
from scipy.ndimage import filters

# OpenCV
import cv2, array
# Ros libraries
import roslib
import rospy



image = cv2.imread('1523243084.44.png')

gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

cv2.imshow('gray_image',gray_image)
cv2.waitKey(0)
cv2.destroyAllWindows()