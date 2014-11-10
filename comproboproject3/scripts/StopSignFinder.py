#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry

import numpy as np
import math
import time


class SignWatcher:
    def __init__(self, verbose = False):
        
        self.stop_detected = 0 # Trying out a 0 definite no stop sign, 5 definite stop sign.  1 probably, 2 possibly not, 3 possibly, 4 probably
        self.stop_sign_img = cv2.imread("greenSmall.png",0)
        self.sift = cv2.SIFT()
        self.MIN_MATCH_COUNT = 10
        self.image_count = 0

        self.templateKp1, self.templateDes1 = self.sift.detectAndCompute(self.stop_sign_img,None)
        
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.bridge = CvBridge()
        rospy.init_node("sign_found")
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)
        self.sign_found_pub = rospy.Publisher("sign_found", String, queue_size=10)
        # if true, we print what is going on
        self.verbose = verbose

        # most recent raw CV image
        self.cv_image = None
        # thread.start_new_thread(self.stopSignThread, (self.cv_image))

        #subscribe to odometry
        rospy.Subscriber('odom',Odometry,self.odometryCb)
        self.xPosition = -1.0
        self.yPosition = -1.0
        self.image_odom = [self.xPosition, self.yPosition] 
        
        self.dprint("Driver Initiated")

    
    def recieveImage(self,raw_image):
        self.dprint("Image Recieved")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:
            print e

        self.image_count += 1     

        self.image_odom = [self.xPosition, self.yPosition] 
        # print "pic"
        # print self.image_odom
        
        if self.image_count % 4 is 0:
            self.checkStop()
        cv2.imshow('Video2', self.cv_image)

        cv2.waitKey(3)


    def checkStop(self):
        # print "check stop"
        self.dprint("Check for the stopsign here")
        # find the keypoints and descriptors with SIFT
        start = time.time()
        imageKP, imageDes = self.sift.detectAndCompute(self.cv_image, None)

        matches = self.flann.knnMatch(self.templateDes1,imageDes,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
        if len(good)>self.MIN_MATCH_COUNT :
            print "found"
            src_pts = np.float32([ self.templateKp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ imageKP[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = self.stop_sign_img.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            # print dst
            # print pts
            # print dst
            isSquare, meanSide = self.isSquare(dst)          
            if isSquare:  
                stopDist = self.estRange(meanSide)

                cv2.polylines(self.cv_image,[np.int32(dst)],True,255,3)
                
                if self.stop_detected < 3:
                    self.stop_detected += 1
                    if self.stop_detected is 2:
                        print "stop sign found"
                        self.signFound(stopDist)    
                
        else:
            matchesMask = None
            if self.stop_detected > 0:
                self.stop_detected -= 1
                if self.stop_detected is 1:
                    print "stop sign lost"


    def signFound(self, stopDist):
        odom = str(self.image_odom[0]) + ","+ str(self.image_odom[1])
        message = odom + "," + str(0.0254*stopDist)
        self.sign_found_pub.publish(message)
        

    def estRange(self, pixelDist):
        return  90.015400923886219 + -.58834960136704306 * pixelDist + .0012499950650678758 * math.pow(pixelDist,2)

    def isSquare(self, pts):
        # print pts
        sides = []
        sides.append(pts[1][0][1] - pts[0][0][1])
        sides.append(pts[2][0][0] - pts[1][0][0])
        sides.append(pts[2][0][1] - pts[3][0][1])
        sides.append(pts[3][0][0] - pts[0][0][0])
        # print sides
        maxSide = max(sides)
        minSide = min(sides)
        meanSide = np.mean(sides)

        if maxSide > meanSide*1.1 or minSide < meanSide *.9 or meanSide < 1:
            # print "False"
            return False, sides 
        return True, meanSide

    def odometryCb(self,msg):
        self.xPosition = msg.pose.pose.position.x
        self.yPosition = msg.pose.pose.position.y

    def dprint(self, print_message):
        if self.verbose:
            print print_message



def main(args):
    ic = SignWatcher(False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



