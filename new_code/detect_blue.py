# Python program for Detection of a
# specific color(blue here) using OpenCV with Python
import cv2
import numpy as np

# Webcamera no 0 is used to capture the frames


# This drives the program into an infinite loop.
#while(1):
    # Captures the live stream frame-by-frame
def main(arg):
    #print(arg)
    #frame = cv2.imread(arg)
    frame = arg
    # Converts images from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([110,50,50])
    upper_red = np.array([130,255,255])

    # Here we are defining range of bluecolor in HSV
    # This creates a mask of blue coloured
    # objects found in the frame.
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # The bitwise and of the frame and mask is done so
    # that only the blue coloured objects are highlighted
    # and stored in res
    #res = cv2.bitwise_and(frame,frame, mask= mask)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        print(x,y)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #print(center)

        # only proceed if the radius meets a minimum size
        #if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
        #cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
        #cv2.circle(frame, center, 5, (0, 0, 255), -1)
        return center



    # cv2.imshow('frame',frame)
    # print('kjfbejfe')
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # if k == 27:
    #     break

# Destroys all of the HighGUI windows.
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(arg)
