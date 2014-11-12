#!/usr/bin/env python

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import Twist, Vector3
import numpy as np
import scipy
from scipy import ndimage
import math
import time

#sunny 2:50pm
# lower_red2 = np.array([0,136,0])
# upper_red2 = np.array([35,255,255])

#Evening 5:00 pm
# lower_red1 = np.array([00,102,165])
# upper_red1 = np.array([28,205,255])
# lower_red2 = np.array([128,75,116])
# upper_red2 = np.array([197,255,255])




class Driver:
    def __init__(self, verbose = False):
        cv2.namedWindow('image')

        cv2.createTrackbar('lowH','image',0,255,nothing)
        cv2.setTrackbarPos('lowH','image',128)
        cv2.createTrackbar('lowS','image',0,255,nothing)
        cv2.setTrackbarPos('lowS','image',75)
        cv2.createTrackbar('lowV','image',0,255,nothing)
        cv2.setTrackbarPos('lowV','image',113)
        cv2.createTrackbar('highH','image',0,255,nothing)
        cv2.setTrackbarPos('highH','image',197)
        cv2.createTrackbar('highS','image',0,255,nothing)
        cv2.setTrackbarPos('highS','image',255)
        cv2.createTrackbar('highV','image',0,255,nothing)
        cv2.setTrackbarPos('highV','image',255)

        cv2.namedWindow("Image window", 1)

        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)
            
        # most recent raw CV image
        self.cv_image = None

        
    def recieveImage(self,raw_image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:
            print e

        imShape = self.cv_image.shape

        print self.cv_image.shape

        cv2.imshow('Video2', self.cv_image)

        smallCopy = self.cv_image[350:480]

        hsv = cv2.cvtColor(smallCopy, cv2.COLOR_BGR2HSV)

        lowH = cv2.getTrackbarPos('lowH','image')
        lowS = cv2.getTrackbarPos('lowS','image')
        lowV = cv2.getTrackbarPos('lowV','image')
        highH = cv2.getTrackbarPos('highH','image')
        highS = cv2.getTrackbarPos('highS','image')
        highV = cv2.getTrackbarPos('highV','image')

        lower_red2 = np.array([lowH,lowS,lowV])
        upper_red2 = np.array([highH,highS,highV])

        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        filteredImage = np.zeros((imShape[0],imShape[1]), np.uint8)

        filteredImage[350:480] = mask2

        cv2.imshow('Video3', filteredImage)

        cv2.waitKey(3)

def nothing(x):
    pass

def main(args):
    ic = Driver(False)
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
        