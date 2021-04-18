#! /usr/bin/env python

import rospy
import math
import cv2
import numpy as np

from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image


from cv_bridge import CvBridge, CvBridgeError


class Control:

    def __init__(self):

        #Subscribers
        self.sub_cam = rospy.Subscriber('/robot_arm/camera/image_raw', Image, self.get_image)
        self.frame=Image()



        self.img_pub = rospy.Publisher("/image/object_info",Image, queue_size=1 )

        self.pub_joint1 = rospy.Publisher("/robot_arm/joint1_position_controller/command",Float64, queue_size=1)
        self.pub_joint2 = rospy.Publisher("/robot_arm/joint2_position_controller/command",Float64, queue_size=1)
        self.pub_joint3 = rospy.Publisher("/robot_arm/joint3_position_controller/command",Float64, queue_size=1)
        self.pub_joint6 = rospy.Publisher("/robot_arm/joint6_position_controller/command",Float64, queue_size=1)
        self.pub_joint7 = rospy.Publisher("/robot_arm/joint7_position_controller/command",Float64, queue_size=1)


        self.joint1=Float64()
        self.joint2=Float64()
        self.joint3=Float64()
        self.joint6=Float64()
        self.joint7=Float64()

        self.bridge =  CvBridge()#bridge for conversion between ROS and OpenCv
        self.distance_cm=0
        self.pen = False




    def get_image(self,ros_image):
        #Callback of the image displa by the camera of the robot
        self.frame=ros_image

    def inverse_kinematic2(self, x ,y , z, alpha):

        print("Start calculus")
        b = 0.5
        theta234 = alpha
        d1 = 1.5
        d3  = 0.86
        d4=1
        d2 = 1.3
        

        theta1 =  - math.atan2(y, x)

        c1 = math.cos(theta1)
        s1 = math.sin(theta1)

        c234 = math.cos(theta234)
        s234 = math.sin(theta234)

        p1 = x * c1 + y * s1 - d4 * c234
        p2 = z - d4 * s234

        x_sqr = x**2
        y_sqr = y**2

        theta3 = -math.acos((x_sqr + y_sqr + (z - d1 - b)**2 - d2**2 - d3**2) / (2*d2*d3))
        c3 = math.cos(theta3)
        s3 = math.sin(theta3)


        theta2 =  math.atan2((z - d1 - b), math.sqrt(x_sqr + y_sqr)) - math.atan2((s3 * d3), (d2 + c3 * d3))
        
        theta4 = alpha

        print("theta1 = " +str(theta1))
        print("theta2 = " +str(theta2))
        print("theta3 = " +str(theta3))
        print("theta4 = " +str(theta4))


        self.pub_joint1.publish(theta1)
        self.pub_joint2.publish(theta2)
        self.pub_joint3.publish(theta3)
        self.pub_joint6.publish(theta4)

        self.joint1.data = theta1
        self.joint2.data = theta2
        self.joint3.data = theta3
        self.joint6.data = theta4


    def computer_vision(self):

        #try and catch for the conversion of a ROS Image in OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.frame, "bgr8")
        except CvBridgeError as e:
            print(e)

        print 'read the image'
        #cv2.imshow("cam",cv_image)
        

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        cv2.imshow("hsv",hsv)

        # Red color
        low_red = (161, 155, 84)
        high_red = (179, 255, 255)

        #find upper and lower vounds of the yellow
        yellowLower=(30, 80, 90)
        yellowUpper=(60,255,255)

        #mask
        mask=cv2.inRange(hsv, yellowLower, yellowUpper)
        #cv2.imshow("mask image",mask)


        x,contour, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #calculate the center of the contour

    
        black_image = np.zeros([mask.shape[0], mask.shape[1], 3], np.uint8)

        if len(contour) > 0:
            self.pen = True
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # determine the center
            c = max(contour, key=cv2.contourArea)
            #print("c= "+str(c))
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            #calculate the center.
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size

            
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(cv_image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(black_image, (int(x), int(y)), int(radius),(255, 255, 255), -1)

            #Focal length of the camera of the robot
            Focal_length_mm=3.04
            #Conversion of the Focal length of the camera based on the focal length in mm/ the size of the sensor and the dimension of the image (all in horizontal)
            Focal_length_pixel=(3.04/3.68)*640
            
            diam=radius*2
            real_size= 0.65
            self.distance_cm = ((real_size*Focal_length_pixel)/diam)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image,'Distance'+str(self.distance_cm), (center[0],center[1]), font ,0.5, (255,0,0),2)

            cv2.imshow("im",cv_image)
            cv2.imshow("black_im",black_image)
        elif len(contour)<0:
            self.pen= False            



        cv2.waitKey(0)
        cv2.destroyAllWindows()



    def close_gripper(self):
        print(self.distance_cm)

        if(self.distance_cm>1):
            print("distance is adequate to grab")
            self.pub_joint7.publish(-0.1)
            self.joint7.data=-0.1
        elif(self.distance<1):
            print("to close to the object")
            self.joint1.data = 0.0
            self.pub_joint1.publish(self.joint1)
            print("Return initial pose")
            self.joint2.data = 0.0
            self.pub_joint2.publish(self.joint2)
            self.joint3.data = 0.0
            self.pub_joint3.publish(self.joint3)
            self.joint6.data = 0.0
            self.pub_joint6.publish(self.joint6)
            self.joint7.data = 0.0
            self.pub_joint7.publish(self.joint7)
        elif(self.distance>3):
            print("to far from the object")
            self.joint1.data = 0.0
            self.pub_joint1.publish(self.joint1)
            print("Return initial pose")
            self.joint2.data = 0.0
            self.pub_joint2.publish(self.joint2)
            self.joint3.data = 0.0
            self.pub_joint3.publish(self.joint3)
            self.joint6.data = 0.0
            self.pub_joint6.publish(self.joint6)
            self.joint7.data = 0.0
            self.pub_joint7.publish(self.joint7)




    def basic(self):

        print("Open gripper")

        basic_pause = 6
        x = 0.4
        y = 0.4
        z = 0.4
        alpha= -0.8

        px = 0.3
        py = -0.8
        pz = 0.4
        alpha= -0.8

        self.pub_joint7.publish(-0.5)
        rospy.sleep(basic_pause)

        print("Go to take pen")
        self.inverse_kinematic2(x, y, z, alpha)
        rospy.sleep(basic_pause)
        self.computer_vision()
        #self.open_gripper()
        rospy.sleep(basic_pause)
        self.close_gripper()

        
        '''self.pub_joint2.publish(-0.6)
        self.joint2.data = -0.6
        print("Close gripper")
        self.pub_joint7.publish(-0.1)
        self.joint7.data = -0.1'''
        if (self.pen ==True):
            rospy.sleep(basic_pause)
            print("Raise up")
            self.pub_joint2.publish(0)
            rospy.sleep(basic_pause)
            print("Go to the support")
            self.inverse_kinematic2(px, py, pz, alpha)
            self.pub_joint2.publish(-0.52)
            rospy.sleep(10)
            
            self.pub_joint2.publish(-0.7)
            rospy.sleep(3)
            self.pub_joint7.publish(-0.5)
            print("Open gripper")
            rospy.sleep(3)
            self.pub_joint2.publish(-0.1)
            rospy.sleep(3)
            rospy.sleep(10)
        elif (self.pen ==False):
            print("no object")
            shutdown_function()

        


    def shutdown_function(self):

        self.joint1.data = 0.0
        self.pub_joint1.publish(self.joint1)
        print("Shutdown")
        self.joint2.data = 0.0
        self.pub_joint2.publish(self.joint2)
        self.joint3.data = 0.0
        self.pub_joint3.publish(self.joint3)
        self.joint6.data = 0.0
        self.pub_joint6.publish(self.joint6)
        self.joint7.data = 0.0
        self.pub_joint7.publish(self.joint7)

        
if __name__ == '__main__':
    rospy.init_node('robot0_node')
    robot0 = Control()
    #robot0.zero()
    #robot0.open_gripper()
    rospy.sleep(2)
    rospy.on_shutdown(robot0.shutdown_function)
    '''robot0.init_camera()
    robot0.open_gripper()
    rospy.on_shutdown(robot0.shutdown_function)
    robot0.inverse_kinematic2(0.199, 0.001)'''
    
    while not rospy.is_shutdown():
        
        #robot0.wave()
        #robot0.basic_xy_ckeck()
        #robot0.basic_quadrant_ckeck()
        robot0.basic()
        #robot0.image_processing()
        rospy.sleep(2)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    