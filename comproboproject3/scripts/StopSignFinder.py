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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion

from geometry_msgs.msg import Twist, Vector3
import numpy as np
import scipy
from scipy import ndimage
import math
import time
import thread

def nothing(x):
    pass

class Driver:
    def __init__(self, verbose = False):
        cv2.namedWindow('image')

        self.ang = 0
        self.road_detected = False
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

        #cv2.namedWindow("Image window", 1)

        self.bridge = CvBridge()
        rospy.init_node("sign_found")
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)
        self.sign_found_pub = rospy.Publisher("sign_found", String, queue_size=10)
        
        # if true, we print what is going on
        self.verbose = verbose

        # most recent raw CV image
        self.cv_image = None
        # thread.start_new_thread(self.stopSignThread, (self.cv_image))

        self.road_detected = False
        self.stop_detected = False

        self.dprint("Driver Initiated")

    
    def recieveImage(self,raw_image):
        self.dprint("Image Recieved")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:

            print e
        self.image_count += 1
        # self.sign_found_pub.publish("HIShANE")
        
        # s = cv2.getTrackbarPos(self.switch,'image')
        
        
        if self.image_count % 2 is 0:
            self.checkStop()
        # cv2.imshow('Video2', self.cv_image)

        # if s == 0:
        #     self.dprint("Driving off")
        #     cv2.waitKey(3)
        #     return


        #cv2.imshow('Video1', self.cv_image)
        # self.followRoad()
    
        #gray= cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        
        # Display the resulting frame

        cv2.waitKey(3)
        #msg = Twist(linear=Vector3(x=.1))
        #self.pub.publish(msg)


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
            if self.isSquare(dst):  
                # print dst[0][0][0]
                top = dst[0][0][1]
                bottom = dst[0][0][1]
                for pt in dst:
                    # print pt[0][1] 
                    if pt[0][1] > bottom:
                        bottom = pt[0][1] 
                    if pt[0][1] < top:
                        top = pt[0][1]
                # print "top" 
                # # print top 
                # # print bottom 
                height = bottom - top
                stopDist = self.estRange(height)
                # print "Dist:"+ str(stopDist)

            # cv2.polylines(self.scene,[np.int32(dst)],True,255,3)
            # print img2
            
            if self.stop_detected < 5:
                self.stop_detected += 1
                if self.stop_detected is 4:
                    print "stop sign found"
                    # cv2.setTrackbarPos(self.switch,'image',0)
                    self.sign_found_pub.publish("HIShANE")
        
                
            # if self.stop_detected > 3:
            #     print "stop sign found"
                
        else:
            # print "Not enough matches are found - %d/%d" % (len(good),self.MIN_MATCH_COUNT )
            matchesMask = None
            if self.stop_detected > 0:
                self.stop_detected -= 1
                if self.stop_detected is 1:
                    print "stop sign lost"

        # print "took:" + str(time.time()- start)
    def estRange(self, pixelDist):
        return  90.015400923886219 + -.58834960136704306 * pixelDist + .0012499950650678758 * math.pow(pixelDist,2)

    def isSquare(self, pts):
        sides = []
        sides.append(pts[1][0][1] - pts[0][0][1])
        sides.append(pts[2][0][0] - pts[1][0][0])
        sides.append(pts[2][0][1] - pts[3][0][1])
        sides.append(pts[3][0][0] - pts[0][0][0])
        maxSide = max(sides)
        minSide = min(sides)
        meanSide = np.mean(sides)

        if maxSide > meanSide*1.1 or minSide < meanSide *.9:
            return False 
        return True

    def sendCommand(self, lin, ang):

        twist = Twist()
        twist.linear.x = lin; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = ang
        self.pub.publish(twist)

    def dprint(self, print_message):
        if self.verbose:
            print print_message



def main(args):
    ic = Driver(False)
    # rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



