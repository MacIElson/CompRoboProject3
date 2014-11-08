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
        self.stop_detected = 0 # Trying out a 0 definite no stop sign, 5 definite stop sign.  1 probably, 2 possibly not, 3 possibly, 4 probably
        self.object_detected = False
        self.stop_sign_img = cv2.imread("greenSmall.png",0)
        self.sift = cv2.SIFT()
        self.MIN_MATCH_COUNT = 10
        self.image_count = 0

        self.dprint("Driver Initiated")
        
    def recieveImage(self,raw_image):
        self.image_count += 1
        self.dprint("Image Recieved")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:
            print e

        self.followRoad()
        self.checkObject()
        if self.image_count % 10 is 0:
            self.checkStop(self.cv_image)
    

        
        gray= cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        
        # Display the resulting frame
        cv2.imshow('Video', self.cv_image)

        cv2.waitKey(3)
        # self.move(Vector3(x=0), Vector3(z=-1))
    
    def move(self, linear_vector, angular_vector = None ):        
        msg = Twist(linear= linear_vector, angular = angular_vector)
        self.pub.publish(msg)
    
    def followRoad(self):
        self.dprint("Follow the road here")
        # gray = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        # edges = cv2.Canny(gray,50,150,apertureSize = 3)

        # lines = cv2.HoughLines(edges,1,np.pi/180,200)
        # if lines is not None:
        #     for rho,theta in lines[0]:
        #         a = np.cos(theta)
        #         b = np.sin(theta)
        #         x0 = a*rho
        #         y0 = b*rho
        #         x1 = int(x0 + 1000*(-b)
        #         y1 = int(y0 + 1000*(a))
        #         x2 = int(x0 - 1000*(-b))
        #         y2 = int(y0 - 1000*(a))

        #         cv2.line(self.cv_image,(x1,y1),(x2,y2),(0,0,255),2)


    def checkObject(self):
        self.dprint("Check for the object here")


    def checkStop(self, scene):
        self.dprint("Check for the stopsign here")
        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(self.stop_sign_img,None)
        kp2, des2 = self.sift.detectAndCompute(scene,None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
        if len(good)>self.MIN_MATCH_COUNT :
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = self.stop_sign_img.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)

            cv2.polylines(scene,[np.int32(dst)],True,255,3)
            # print img2
            if self.stop_detected < 5:
                self.stop_detected += 1
                if self.stop_detected is 4:
                    print "stop sign found"
                
            # if self.stop_detected > 3:
            #     print "stop sign found"
                
        else:
            # print "Not enough matches are found - %d/%d" % (len(good),self.MIN_MATCH_COUNT )
            matchesMask = None
            if self.stop_detected > 0:
                self.stop_detected -= 1
                if self.stop_detected is 1:
                    print "stop sign lost"
            

    def dprint(self, print_message):
        if self.verbose:
            print print_message

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


# self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
