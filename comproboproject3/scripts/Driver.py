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

        self.timeLost = -1

        cv2.createTrackbar('speed','image',0,200,nothing)
        cv2.setTrackbarPos('speed','image',00)

        cv2.createTrackbar('pidP','image',0,8000,nothing)
        cv2.setTrackbarPos('pidP','image',400)

        cv2.createTrackbar('pidI','image',0,400,nothing)
        cv2.setTrackbarPos('pidI','image',20)

        cv2.createTrackbar('pidD','image',0,4000,nothing)
        cv2.setTrackbarPos('pidD','image',370)

        cv2.createTrackbar('lowH','image',0,255,nothing)
        cv2.setTrackbarPos('lowH','image',0)
        cv2.createTrackbar('lowS','image',0,255,nothing)
        cv2.setTrackbarPos('lowS','image',96)
        cv2.createTrackbar('lowV','image',0,255,nothing)
        cv2.setTrackbarPos('lowV','image',150)
        cv2.createTrackbar('highH','image',0,255,nothing)
        cv2.setTrackbarPos('highH','image',20)
        cv2.createTrackbar('highS','image',0,255,nothing)
        cv2.setTrackbarPos('highS','image',255)
        cv2.createTrackbar('highV','image',0,255,nothing)
        cv2.setTrackbarPos('highV','image',255)

        self.switch = '0 : OFF \n1 : ON'
        cv2.createTrackbar(self.switch, 'image',0,1,self.stop)
        cv2.setTrackbarPos(self.switch,'image',0)

        self.pid = PID(P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500)
        self.pid.setPoint(float(320))

        #cv2.namedWindow("Image window", 1)

        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sign_found_sub = rospy.Subscriber('sign_found', String, self.stopSignFound)
            
        # if true, we print what is going on
        self.verbose = verbose

        # most recent raw CV image
        self.cv_image = None
        # thread.start_new_thread(self.stopSignThread, (self.cv_image))

        self.road_detected = False
        self.stop_detected = False

        self.dprint("Driver Initiated")

    def stop(self, x):
        if x == 0:
            self.sendCommand(0,0)


    def stopSignFound(self, message):
        print "YAYAYYA"
        print message
    
    def recieveImage(self,raw_image):
        self.dprint("Image Recieved")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:

            print e
        self.image_count += 1
        
        s = cv2.getTrackbarPos(self.switch,'image')
        
        self.sign_found_sub = rospy.Subscriber('/sign_found', String, queue_size=10)
        
        # if self.image_count % 5 is 0:
            # thread.start_new_thread(self.checkStop, (self.cv_image,))

            # self.checkStop()
        cv2.imshow('Video2', self.cv_image)

        if s == 0:
            self.dprint("Driving off")
            cv2.waitKey(3)
            return


        #cv2.imshow('Video1', self.cv_image)
        self.followRoad()
    
        #gray= cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        
        # Display the resulting frame

        cv2.waitKey(3)
        #msg = Twist(linear=Vector3(x=.1))
        #self.pub.publish(msg)

    def followRoad(self):
        workingCopy = self.cv_image
        imShape = workingCopy.shape

        smallCopy = workingCopy[350:480]

        hsv = cv2.cvtColor(smallCopy, cv2.COLOR_BGR2HSV)

        pidP100 = cv2.getTrackbarPos('pidP','image')
        pidI100 = cv2.getTrackbarPos('pidI','image')
        pidD100 = cv2.getTrackbarPos('pidD','image')

        pidP = float(pidP100)/100
        pidI = float(pidI100)/100
        pidD = float(pidD100)/100

        self.pid.setKp(pidP)
        self.pid.setKi(pidI)
        self.pid.setKd(pidD)

        lower_red1 = np.array([00,102,165])
        upper_red1 = np.array([28,205,255])

        lowH = cv2.getTrackbarPos('lowH','image')
        lowS = cv2.getTrackbarPos('lowS','image')
        lowV = cv2.getTrackbarPos('lowV','image')
        highH = cv2.getTrackbarPos('highH','image')
        highS = cv2.getTrackbarPos('highS','image')
        highV = cv2.getTrackbarPos('highV','image')


        lower_red2 = np.array([lowH,lowS,lowV])
        upper_red2 = np.array([highH,highS,highV])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = cv2.bitwise_or(mask1, mask2)

        mask = mask2

        filteredImage = np.zeros((imShape[0],imShape[1]), np.uint8)

        num = []

        #sum all coplumns into 1 row
        driveRow = np.sum(mask,0)
        

        for i in range(len(driveRow)):
            if driveRow[i] > 0:
                num.append(i+1)
        
        speed100 = cv2.getTrackbarPos('speed','image')
        speed = float(speed100)/100

        if len(num) == 0:
            ang = math.copysign(1, self.ang) * max(.5, min(1, abs(self.ang)))
            self.sendCommand(.1, ang)
        else:
            averageLineIndex = (float(sum(num))/len(num))
            ang = self.pid.update(averageLineIndex)/1000
            self.sendCommand(speed, ang)
        self.ang = ang
        #print "ang: " + str(ang)

        filteredImage[350:480] = mask

        cv2.imshow('Video3', filteredImage)
    
    def stopSignThread(self):
        print "hi"
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                print "hi2"
                self.checkStop()
                print ""

    def checkStop(self, scene):
        print "check stop"
        self.dprint("Check for the stopsign here")
        # find the keypoints and descriptors with SIFT
        start = time.time()
        imageKP, imageDes = self.sift.detectAndCompute(scene, None)

        matches = self.flann.knnMatch(self.templateDes1,imageDes,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
        if len(good)>self.MIN_MATCH_COUNT :
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
                    cv2.setTrackbarPos(self.switch,'image',0)

                
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


class PID:
    """
    Discrete PID control
    source: http://code.activestate.com/recipes/577231-discrete-pid-controller/
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

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
