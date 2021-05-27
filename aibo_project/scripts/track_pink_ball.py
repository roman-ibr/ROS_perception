#!/usr/bin/env python

#===============================================================
#Name and Surname: Roman Ibrahimov
#Email address: ibrahir@purdue.edu
#Program Description: 
'''
This program is used to find pink ball around the robot. First, the robot rotates slowly to find the
ball around. When the direction is detected, the robot moves to the ball. When it gets closer, the y position of
the blob on the image chages. Based on this, we detect the distance to the ball.  
The code is based the previous Kobuki robot task. Before this code, please, launch my_mira_cmvision_tc.launch
'''
#=================================================================




import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blobs, Blob
from std_msgs.msg import Float64
# import sensor_msgs.point_cloud2 as pc2
# import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import time
import numpy


#global
turn = 0.0 #turning rate
move = 0.0 #moverment to the ball
tail = 0.0 # flag to turn on wiggling the tail

tailPan_min = 0.0
tailPan_max = 0.0
blob_position = 0 # x position for the blob
blob_position_y = 0

# callback function checks to see if any blobs were found then
# loop through each and get the x position.  Since the camera
# will sometimes find many blobs in the same object we just
# average all the x values.  You could also just take the first
# one if you are sure you will only have one blob. 
#
# This doesn't use multiple blobs but if are tracking several 
# objects you need to check the /data.blobs.color topic for
# the color tag you put in your colors.txt file. 
#
# after we have the x value we just make the robot turn to 
# keep it in the center of the image.

def callback(data):
    global turn
    global move
    global tail
    global blob_position

#For all blobs, we are checking whether the Pink one is there.
#If it is detected, we get x and y positions of the blob.
#If it is smaller than the certain threshold, we try to direct the robot towards the ball

    if(len(data.blobs)):

        for obj in data.blobs:
            if obj.name == "Pink":
                rospy.loginfo("Blob <"+str(obj.name)+"> Detected!")
                blob_position = obj.x
                blob_position_y = obj.y
                print("this is x position", blob_position)
                print("this is y position", blob_position_y)
                # blob_distance =
        
                rospy.loginfo("blob is at %s"%blob_position)
                # turn right if we set off the left cliff sensor
                if( blob_position > 200 ):
                    rospy.loginfo("TURN RIGHT")
                    turn = -0.1
                    move = 0.0 
                # turn left if we set off the right cliff sensor
                if( blob_position < 190 ):
                    rospy.loginfo("TURN LEFT")
                    turn = 0.1
                    move = 0.0 
        
                if( blob_position > 189 and blob_position < 201): # 180 220
                    rospy.loginfo("CENTERED")
                    turn = 0.0
                    if (blob_position_y<160):
                        move = 0.1
                    else: 
                        tail = 1.0
                            
                        # tail = 0.1 
    else: 
        turn = 0.1
        move = 0.0 


def run():
    rospy.init_node("track_blob_color_node", log_level=rospy.WARN)
    global blob_position
    # publish twist messages to /cmd_vel
    pub = rospy.Publisher('/aiboERS7/cmd_vel', Twist, queue_size=1)    
    #subscribe to the robot sensor state
    rospy.Subscriber('/blobs', Blobs, callback)
    # publish Float64 messages to move the tail of the robot
    pub_wiggle = rospy.Publisher('/aibo_tc/tailPan_position_controller/command', Float64, queue_size=4) 


    global turn
    global move
    global tail
    global tailPan_min 
    global tailPan_max 
    twist = Twist()
    x = Float64()
    y = Float64()



    while not rospy.is_shutdown():

        # rotating the robot towards the ball
        if ( turn != 0.0 ):
            str = "Turning %s"%turn
            rospy.loginfo(str)
            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
            # x = turn
            y.data = 0.0 
            turn = 0.0

            tailPan_min = 0.0
            tailPan_max = 0.0


            # if it is towards the ball, it will move straight
        elif (move != 0.0): 
            str = "Turning %s"%turn
            rospy.loginfo(str)
            twist.linear.x = 0.1; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            move = 0.0

            tailPan_min = 0.0
            tailPan_max = 0.0
            #when it is close enough, the robot will stop and wiggling will start
        else:
            str = "Straight %s"%turn
            rospy.loginfo(str)
            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            

            pub.publish(twist)
            rospy.sleep(0.1)
            while (tail!=0.0): 
                pub_wiggle.publish(0.6)
                rospy.sleep(0.5)
                pub_wiggle.publish(-0.6)
                rospy.sleep(0.5)


        pub.publish(twist)
        blob_position = 0
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass