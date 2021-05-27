#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blobs, Blob

#global
turn = 0.0 #turning rate
blob_position = 0 # x position for the blob

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
    global blob_position

    if(len(data.blobs)):

        for obj in data.blobs:
            if obj.name == "RedBall":
                rospy.loginfo("Blob <"+str(obj.name)+"> Detected!")
                blob_position = obj.x
        
                rospy.loginfo("blob is at %s"%blob_position)
                # turn right if we set off the left cliff sensor
                if( blob_position > 220 ):
                    rospy.loginfo("TURN RIGHT")
                    turn = -0.1
                # turn left if we set off the right cliff sensor
                if( blob_position < 180 ):
                    rospy.loginfo("TURN LEFT")
                    turn = 0.1
        
                if( blob_position > 180 and blob_position < 220):
                    rospy.loginfo("CENTERED")
                    turn = 0.0
    else: 
        turn = 0.0

def run():
    rospy.init_node("track_blob_color_node", log_level=rospy.WARN)
    global blob_position
    # publish twist messages to /cmd_vel
    pub = rospy.Publisher('/mira/commands/velocity', Twist, queue_size=1)

    #subscribe to the robot sensor state
    rospy.Subscriber('/blobs', Blobs, callback)
    

    global turn
    twist = Twist()

    while not rospy.is_shutdown():

        # turn if we hit the line
        if ( turn != 0.0 ):
            str = "Turning %s"%turn
            rospy.loginfo(str)
            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
            turn = 0.0

            # straight otherwise
        else:
            str = "Straight %s"%turn
            rospy.loginfo(str)
            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

            # send the message and delay
        pub.publish(twist)
        blob_position = 0
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass