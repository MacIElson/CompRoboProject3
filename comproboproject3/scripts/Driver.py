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

def nothing(x):
    pass

class Driver:
    def __init__(self, verbose = False):
        cv2.namedWindow('image')

        self.road_detected = False
        self.stop_detected = 0 # Trying out a 0 definite no stop sign, 5 definite stop sign.  1 probably, 2 possibly not, 3 possibly, 4 probably
        self.object_detected = False
        self.stop_sign_img = cv2.imread("greenSmall.png",0)
        self.sift = cv2.SIFT()
        self.MIN_MATCH_COUNT = 10
        self.image_count = 0

        cv2.createTrackbar('speed','image',0,200,nothing)
        cv2.setTrackbarPos('speed','image',200)

        cv2.createTrackbar('pidP','image',0,800,nothing)
        cv2.setTrackbarPos('pidP','image',100)

        cv2.createTrackbar('pidI','image',0,400,nothing)
        cv2.setTrackbarPos('pidI','image',0)

        cv2.createTrackbar('pidD','image',0,400,nothing)
        cv2.setTrackbarPos('pidD','image',0)

        cv2.createTrackbar('edgeMin','image',0,100,nothing)
        cv2.setTrackbarPos('edgeMin','image',50)
        cv2.createTrackbar('edgeMax','image',0,300,nothing)
        cv2.setTrackbarPos('edgeMax','image',150)

        cv2.createTrackbar('largeSize','image',0,1000,nothing)
        cv2.setTrackbarPos('largeSize','image',500)

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

        self.pid = PID(P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500)
        self.pid.setPoint(float(280))

        cv2.namedWindow("Image window", 1)

        time.sleep(5)

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
        self.image_count += 1
        self.dprint("Image Recieved")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:
            print e

        cv2.imshow('Video1', self.cv_image)
        self.followRoad3()
        self.checkObject()
        if self.image_count % 10 is 0:
            self.checkStop(self.cv_image)
    

        
        gray= cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        
        # Display the resulting frame
        cv2.imshow('Video2', self.cv_image)

        cv2.waitKey(3)
        #msg = Twist(linear=Vector3(x=.1))
        #self.pub.publish(msg)
        

    def followRoad(self):
        self.dprint("Follow the road here")

        edgeMin = cv2.getTrackbarPos('edgeMin','image')
        edgeMax = cv2.getTrackbarPos('edgeMax','image')

        gray = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        graySmall = gray[350:480, 100:600]
        cv2.imshow('Video3', graySmall)
        edges = cv2.Canny(graySmall,edgeMin,edgeMax,apertureSize = 3)

        shape = gray.shape

        lines = cv2.HoughLines(edges,1,np.pi/180*3,50)
        if lines is not None:
            for rho,theta in lines[0]:
                #print rho,theta
                if self.findifCrossingXAxis(rho,theta,shape[0],shape[1]):
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    #cv2.line(graySmall,(x1,y1),(x2,y2),(128),2)
                    #if theta > 3*math.pi/4 or theta < math.pi/4:
                    cv2.line(graySmall,(x1,y1),(x2,y2),(0),2)

        gray[350:480, 100:600] = graySmall
        self.cv_image = gray

    def findifCrossingXAxis(self,rho,theta,imageY,imageX):
        point = self.findXIntercept(rho,theta,imageY)
        if point > 0 and point < imageX:
            return True
        return False

    def findXIntercept(self,rho,theta,imageY):
        xIntercept = None
        if theta == 0:
            xIntercept = rho*math.cos(theta)
        else: 
            a = (imageY-rho*math.sin(theta))/math.tan(math.pi/2 - theta)
            xIntercept = -1*math.copysign(1, math.cos(theta))*a + rho * math.cos(theta)
        return xIntercept


        


    def followRoad3(self):
        workingCopy = self.cv_image
        imShape = workingCopy.shape
        print imShape

        smallCopy = workingCopy[350:480, 40:600]

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

        lowH = cv2.getTrackbarPos('lowH','image')
        lowS = cv2.getTrackbarPos('lowS','image')
        lowV = cv2.getTrackbarPos('lowV','image')
        highH = cv2.getTrackbarPos('highH','image')
        highS = cv2.getTrackbarPos('highS','image')
        highV = cv2.getTrackbarPos('highV','image')
        speed100 = cv2.getTrackbarPos('speed','image')
        speed = float(speed100)/100

        lower_red3 = np.array([lowH,lowS,lowV])
        upper_red3 = np.array([highH,highS,highV])

        mask3 = cv2.inRange(hsv, lower_red3, upper_red3)

        filteredImage = np.zeros((imShape[0],imShape[1]), np.uint8)

        driveRow = mask3[-1]+mask3[-2]+mask3[-3]+mask3[-4]+mask3[-5]

        num = [];
        
        for i in range(len(driveRow)):
            if driveRow[i] > 0:
                num.append(i+1)

        if len(num) == 0:
            self.sendCommand(0, self.ang)
        else:
            error1 = (float(sum(num))/len(num))
            error2 = (len(driveRow))/2
            error = -1*(error1-error2)

            ang = self.pid.update((float(sum(num))/len(num)))/1000

            print "ang: " + str(ang)

            self.ang = ang

            self.sendCommand(speed, ang)

        filteredImage[350:480, 40:600] = mask3

        self.cv_image = filteredImage

    def followRoad2(self):
        workingCopy = self.cv_image
        imShape = workingCopy.shape
        print imShape
        #lower_green1 = np.array([160,176,00])
        #upper_green1 = np.array([223,223,16])
        smallCopy = workingCopy[350:480, 40:600]

        hsv = cv2.cvtColor(smallCopy, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([00,102,165])
        upper_red1 = np.array([28,205,255])

        lower_red2 = np.array([00,00,00])
        upper_red2 = np.array([00,00,00])

        lowH = cv2.getTrackbarPos('lowH','image')
        lowS = cv2.getTrackbarPos('lowS','image')
        lowV = cv2.getTrackbarPos('lowV','image')
        highH = cv2.getTrackbarPos('highH','image')
        highS = cv2.getTrackbarPos('highS','image')
        highV = cv2.getTrackbarPos('highV','image')
        speed100 = cv2.getTrackbarPos('speed','image')
        speed = float(speed100)/100

        lower_red3 = np.array([lowH,lowS,lowV])
        upper_red3 = np.array([highH,highS,highV])

        mask1 = cv2.inRange(smallCopy, lower_red1, upper_red1)

        mask2 = cv2.inRange(smallCopy, lower_red2, upper_red2)

        mask3 = cv2.inRange(hsv, lower_red3, upper_red3)

        mask12 = cv2.bitwise_or(mask1, mask2)

        mask123 = cv2.bitwise_or(mask12, mask3)

        filteredImage = np.zeros((imShape[0],imShape[1]), np.uint8)

        driveRow = mask3[-1]+mask3[-2]+mask3[-3]+mask3[-4]+mask3[-5]

        num = [];
        #print driveRow
        for i in range(len(driveRow)):
            if driveRow[i] > 0:
                num.append(i+1)

        if len(num) == 0:
            self.sendCommand(0, self.ang)
        else:
            error1 = (float(sum(num))/len(num))
            error2 = (len(driveRow))/2
            error = -1*(error1-error2)
            ang = float(error*math.pow(abs(error),.6))/(400/speed)

            ang  = max(-3.5*speed, min(3.5*speed, ang))

            print float(sum(num))/len(num)

            print "error1: " + str(error1)
            print "error2: " + str(error2)
            print "error: " + str(error)
            print "ang: " + str(ang)

            self.ang = ang

            self.sendCommand(speed, ang)

        filteredImage[350:480, 40:600] = mask3

        self.cv_image = filteredImage


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
