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


class Driver:

    def __init__(self, verbose = False):
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            
        # if true, we print what is going on
        self.verbose = verbose

        # most recent raw CV image
        self.cv_image = None

        self.road_detected = False
        self.stop_detected = False
        self.object_detected = False

        self.dprint("Driver Initiated")
        
    def recieveImage(self,raw_image):
        self.dprint("Image Recieved")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:
            print e

        self.followRoad()
        self.checkObject()
        self.checkStop()
    

    
        # Display the resulting frame
        cv2.imshow('Video', self.cv_image)

        cv2.waitKey(3)
        msg = Twist(linear=Vector3(x=.1))
        self.pub.publish(msg)
        

    def followRoad(self):
        self.dprint("Follow the road here")
        gray = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)

        lines = cv2.HoughLines(edges,1,np.pi/180,200)
        if lines is not None:
            for rho,theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))

                cv2.line(self.cv_image,(x1,y1),(x2,y2),(0,0,255),2)


    def checkObject(self):
        self.dprint("Check for the object here")


    def checkStop(self):
        self.dprint("Check for the stopsign here")

    def dprint(self, print_message):
        if self.verbose:
            print print_message

def main(args):
    ic = Driver(True)
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


# self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
