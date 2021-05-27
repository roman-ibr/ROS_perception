#!/usr/bin/env python

#===============================================================
#Name and Surname: Roman Ibrahimov
#Email address: ibrahir@purdue.edu
#Program Description: 
'''
This program is used to navigate Aibo robot through the white line. When the robot detects green star, it stops. 
The code is based the previous Kobuki robot task.
'''
#=================================================================





import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_aibo import AiboJointMover


class LineFollower(object):

    def __init__(self):
        #ROS OpenCV bridge object
        self.bridge_object = CvBridge()
        #subscription to the main camera for OpenCV
        self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.camera_callback)
        #velocity publisher for robot
        self.vel_pub = rospy.Publisher('/aiboERS7/cmd_vel', Twist, queue_size=1)    
        #this is object from move_aibo.py. Actually, I did not need it to be used
        self.movekobuki_object = AiboJointMover()
        # the flag to stop the robot when the stat was detected
        self.flag = 1

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make the process faster.
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 20
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # Define the Yellow Colour in HSV
        #[[[ 30 196 235]]]
        """
        To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
        >>> yellow = np.uint8([[[B,G,R ]]])
        >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >>> print( hsv_yellow )
        [[[ 60 255 255]]]
        """
        #upper and lower boundaries for white path
        lower_white = np.array([0,0,185]) #0,0,110])165
        upper_white = np.array([218, 255, 255])





        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        
        contours, _, ___ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        # rospy.loginfo("Number of centroids==>"+str(len(contours))) ################################
        centres = []
        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            try:
                centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
                cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)
            except ZeroDivisionError:
                pass
            
        
        # rospy.loginfo(str(centres)) #############
        
        
        
        # Draw the centroid in the resulting image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        
        

        cv2.imshow("Original", cv_image)
        #cv2.imshow("HSV", hsv)
        #cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)
        
        cv2.waitKey(1)

        #when we move from the white path to the green star, the values of centroids changes
        #it is detected and the robot stops based on this 
        if(cy!=9.0 and self.flag ==1):
            error_x = cx - width / 2;
            twist_object = Twist();
            twist_object.linear.x = 0.2;
            twist_object.angular.z = -error_x / 100;
        else: 
            twist_object = Twist();
            twist_object.linear.x = 0.0;
            twist_object.angular.z = 0.0
            rospy.loginfo("Star was detected!")
            self.flag=0



        # rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))##################################3
        # Make it start turning

        self.vel_pub.publish(twist_object)
        # self.movekobuki_object.move_robot(twist_object)
        
        
    def clean_up(self):
        self.movekobuki_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    
    line_follower_object = LineFollower()

    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    main()