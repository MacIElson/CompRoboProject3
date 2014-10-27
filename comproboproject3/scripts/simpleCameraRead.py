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

import numpy as np

class image_reader:

    def __init__(self):
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.convertImage)
        # self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size =1)
        print "making"
        # self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.callback)

    def convertImage(self,data):
        print "callback"
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
    
        gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
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

                cv2.line(cv_image,(x1,y1),(x2,y2),(0,0,255),2)

    
        # Display the resulting frame
        cv2.imshow('Video', cv_image)


        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (50,50), 10, 255)

        # cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


def main(args):
  ic = image_reader()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


# self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
