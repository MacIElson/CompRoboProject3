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

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix, quaternion_from_euler
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Vector3
import numpy as np
import math
import thread

def nothing(x):
    pass

class Driver:
    def __init__(self, verbose = False):
        rospy.init_node('comproboproject3', anonymous=True)
        cv2.namedWindow('image')

        self.ang = 0
        self.image_count = 0

        self.timeLost = -1

        self.base_frame = "base_link"       # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"

        self.tf_listener = TransformListener()

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

        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)

        #subscribe to odometry
        rospy.Subscriber('odom',Odometry,self.odometryCb)
        self.xPosition = -1.0
        self.yPosition = -1.0


        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sign_found_sub = rospy.Subscriber('sign_found', String, self.stopSignFound)
            
        # if true, we print what is going on
        self.verbose = verbose

        # most recent raw CV image
        self.cv_image = None
        self.stopSignFoundx = -1
        self.stopSignFoundy = -1
        self.stopSignFoundDist = -1

        self.dprint("Driver Initiated")

    def stop(self, x):
        if x == 0:
            self.sendCommand(0,0)

    #callback for when stop sign found
    def stopSignFound(self, message):
        data = message.split(",")
        self.stopSignFoundx = float(data[0])
        self.stopSignFoundy = float(data[1])
        self.stopSignFoundDist = float(data[2])
        
        print message
        cv2.setTrackbarPos(self.switch,'image',0)
    
    #odometry callback
    def odometryCb(self,msg):
        self.xPosition = msg.pose.pose.position.x
        self.yPosition = msg.pose.pose.position.y

    def euclidDistance(self,x1,y1,x2,y2):
        return math.hypot(x2 - x1, y2 - y1)
        
    def recieveImage(self,raw_image):
        self.dprint("Image Recieved")

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:
            print e
                    
        s = cv2.getTrackbarPos(self.switch,'image')
        
        cv2.imshow('Video2', self.cv_image)

        if s == 0:
            self.dprint("Driving off")
            cv2.waitKey(3)
            return


        self.followRoad()
    
        
        cv2.waitKey(3)

    def checkDistToStop(self):
        if not self.stopSignFoundDist == -1:
            currentDist = euclidDistance(self.xPosition,self.yPosition,self.stopSignFoundx,self.stopSignFoundy)
            if abs(self.stopSignFoundDist - currentDist) < .05:
                cv2.setTrackbarPos(self.switch,'image',0)


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

    def sendCommand(self, lin, ang):
        print lin,ang
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
    r = rospy.Rate(60)

    while not(rospy.is_shutdown()):
        # in the main loop all we do is continuously broadcast the latest map to odom transform
        r.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


# self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
