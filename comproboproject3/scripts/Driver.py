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
from std_msgs.msg import Header, String
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Vector3
import numpy as np
import math

#use colorCalibration.py to find the correct color range for the red line
#connect to neato --> run this and StopSignFinder --> turn robot to on

#function that does nothing
def nothing(x):
    pass

class Driver:
    def __init__(self, verbose = False):
        rospy.init_node('comproboproject3', anonymous=True)
        cv2.namedWindow('image')

        #initialize variables
        self.ang = 0
        self.image_count = 0


        # if true, we print what is going on
        self.verbose = verbose

        # most recent raw CV image
        self.cv_image = None

        #initialize StopSignValues
        self.stopSignFoundx = -1
        self.stopSignFoundy = -1
        self.stopSignFoundDist = -1

        #Initialize Sliders for parameters and set initial value

        cv2.createTrackbar('speed','image',0,200,nothing)
        cv2.setTrackbarPos('speed','image',70)

        cv2.createTrackbar('pidP','image',0,8000,nothing)
        cv2.setTrackbarPos('pidP','image',430)

        cv2.createTrackbar('pidI','image',0,400,nothing)
        cv2.setTrackbarPos('pidI','image',20)

        cv2.createTrackbar('pidD','image',0,4000,nothing)
        cv2.setTrackbarPos('pidD','image',385)

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

        #create on/off switch for robot, defaulted to off
        self.switch = '0 : OFF \n1 : ON'
        cv2.createTrackbar(self.switch, 'image',0,1,self.stop)
        cv2.setTrackbarPos(self.switch,'image',0)

        #initialize PID controller
        self.pid = PID(P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500)
        self.pid.setPoint(float(320))

        self.bridge = CvBridge()
        
        #subscribe tocamera images
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)

        #subscribe to odometry
        rospy.Subscriber('odom',Odometry,self.odometryCb)
        self.xPosition = -1.0
        self.yPosition = -1.0

        #set up publisher to send commands to the robot
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #subscribe to the stopSignFinder node (can function without it but will not stop at stop signs)
        self.sign_found_sub = rospy.Subscriber('sign_found', String, self.stopSignFound)


        self.dprint("Driver Initiated")

    #function that stops the robot is 0 is passed in, primary use is call back from stop switch
    def stop(self, x):
        if x == 0:
            self.sendCommand(0,0)

    #callback for when stop sign found
    def stopSignFound(self, message):

        data = message.data.split(",")
        self.stopSignFoundx = float(data[0])
        self.stopSignFoundy = float(data[1])
        self.stopSignFoundDist = float(data[2])
        
        print message
        #cv2.setTrackbarPos(self.switch,'image',0)
    
    #odometry callback
    def odometryCb(self,msg):
        self.xPosition = msg.pose.pose.position.x
        self.yPosition = msg.pose.pose.position.y

    #calculate distance between 2 points
    def euclidDistance(self,x1,y1,x2,y2):
        return math.hypot(x2 - x1, y2 - y1)
        
    # callback when image recieved
    def recieveImage(self,raw_image):
        self.dprint("Image Recieved")

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:
            print e
                   
        #display image recieved
        cv2.imshow('Video2', self.cv_image)

        #determine if we have reached the last detected stop sign
        self.checkDistToStop()

        #retreive value of on/off switch
        s = cv2.getTrackbarPos(self.switch,'image')
        
        #determine if robot should move
        if s == 0:
            self.dprint("Driving Stopped")
        else:
            self.followRoad()
        
        #wait for openCV elements to refresh
        cv2.waitKey(3)

    #check if the robot has reached the last detected stop sign
    def checkDistToStop(self):
        #check if a Stop Sign has been detected
        if not self.stopSignFoundDist == -1:
            #calulate disance that the robot has moved since the stop sign has been detected (.10 added to account for stop response delay)
            currentDist = self.euclidDistance(self.xPosition,self.yPosition,self.stopSignFoundx,self.stopSignFoundy) + .10

            #print distance to stop sign
            print "haveDist" + str(self.stopSignFoundDist - currentDist)

            #if in range of stop sign
            if abs(self.stopSignFoundDist - currentDist) < .04:
                print "robot STOP"
                #stop robot
                self.sendCommand(0,0)

                #tell robot it's off
                cv2.setTrackbarPos(self.switch,'image',0)

                #reset stopsign variables
                self.stopSignFoundDist = -1
                self.stopSignFoundy = -1
                self.stopSignFoundx = -1


    def followRoad(self):
        workingCopy = self.cv_image

        #find image shape
        imShape = workingCopy.shape

        #look at lower segment of image (just the road)
        smallCopy = workingCopy[350:480]

        #convert smallColpy to hsv
        hsv = cv2.cvtColor(smallCopy, cv2.COLOR_BGR2HSV)

        #get and set PID control constants
        pidP100 = cv2.getTrackbarPos('pidP','image')
        pidI100 = cv2.getTrackbarPos('pidI','image')
        pidD100 = cv2.getTrackbarPos('pidD','image')

        pidP = float(pidP100)/100
        pidI = float(pidI100)/100
        pidD = float(pidD100)/100

        self.pid.setKp(pidP)
        self.pid.setKi(pidI)
        self.pid.setKd(pidD)

        #define boundaries for color detection

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

        #perform color thresholding
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        #combine masks
        mask = cv2.bitwise_or(mask1, mask2)

        #ignore first mask (depends on lighting conditions if one or two filters needed)
        mask = mask2

        #sum all coplumns into 1 row
        driveRow = np.sum(mask,0)
        
        #initialize array of x indicies where the road exists
        num = []

        #fill array num with x indicies where the road exists
        for i in range(len(driveRow)):
            if driveRow[i] > 0:
                num.append(i+1)
        
        #compute the speed the robot should travel at
        speed100 = cv2.getTrackbarPos('speed','image')
        speed = float(speed100)/100


        if len(num) == 0:
            #if no road/line found

            #calculate drive row from previous image
            driveRow = np.sum(self.previousMask,0)

            #fill array num with x indicies where the road exists in last good image
            for i in range(len(driveRow)):
                if driveRow[i] > 0:
                    num.append(i+1)

            #calculate average road position
            averageLineIndex = (float(sum(num))/len(num))

            #determine direction robot should turn
            if averageLineIndex <= 320:
                sign = 1
            else:
                sign = -1

            #force robot to turn the correct direction at a speed between 1.5 and 2
            ang = sign * max(1.5, min(2, abs(self.ang)))

            #send turn command
            self.sendCommand(.2, ang)
        else:
            #if line found
            averageLineIndex = (float(sum(num))/len(num))

            #use the pid controller to determine the anglular velocity
            ang = self.pid.update(averageLineIndex)/1000

            #send the move command to the robot
            self.sendCommand(speed, ang)
            #save current mask in case the next image conatins no raod
            self.previousMask = mask
        self.ang = ang
        #print "ang: " + str(ang)

        #create filtered image to display
        filteredImage = np.zeros((imShape[0],imShape[1]), np.uint8)
        filteredImage[350:480] = mask

        #show filtered image
        cv2.imshow('Video3', filteredImage)

    #send movement command to robot
    def sendCommand(self, lin, ang):
        twist = Twist()
        twist.linear.x = lin; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = ang
        self.pub.publish(twist)

    #function that makes print statements switchable
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
    # initialize driver
    ic = Driver(False)

    #set ROS refresh rate
    r = rospy.Rate(60)

    #keep program running until shutdown
    while not(rospy.is_shutdown()):
        r.sleep()

    #close all windows on exit
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


# self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
