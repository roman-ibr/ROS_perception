#!/usr/bin/env python
import time
import rospy
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
"""
Topics To Write on:
type: std_msgs/Float64
/mira/pitch_joint_position_controller/command
/mira/roll_joint_position_controller/command
/mira/yaw_joint_position_controller/command
"""

class MiraBlobTracker(object):

    def __init__(self):
        
        rospy.loginfo("Mira JointMover Initialising...")
        self.pub_mira_roll_joint_position = rospy.Publisher('/mira/roll_joint_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.pub_mira_pitch_joint_position = rospy.Publisher('/mira/pitch_joint_position_controller/command',
                                                             Float64,
                                                             queue_size=1)
        self.pub_mira_yaw_joint_position = rospy.Publisher('/mira/yaw_joint_position_controller/command',
                                                           Float64,
                                                           queue_size=1)
        joint_states_topic_name = "/mira/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.mira_joints_callback)
        mira_joints_data = None
        while mira_joints_data is None:
            try:
                mira_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass

        self.mira_joint_dictionary = dict(zip(mira_joints_data.name, mira_joints_data.position))
        print self.mira_joint_dictionary
        rospy.Subscriber('/mira/commands/velocity',  Twist, self.blob_info_callback)
        
        
    def blob_info_callback(self, msg):
        rospy.loginfo("Blob info Detected==>"+str(msg.angular.z))
        turn_value = msg.angular.z
        yaw_actual_pos = self.mira_joint_dictionary.get("yaw_joint")
        next_value = yaw_actual_pos + turn_value
        rospy.loginfo("Move Head to Blob==>"+str(next_value))
        #self.move_mira_yaw_joint(position=next_value)
        self.ove_mira_all_joints(roll=0.0, pitch=0.0, yaw=next_value)

    def move_mira_all_joints(self, roll, pitch, yaw):
        angle_roll = Float64()
        angle_roll.data = roll
        angle_pitch = Float64()
        angle_pitch.data = pitch
        angle_yaw = Float64()
        angle_yaw.data = yaw
        self.pub_mira_roll_joint_position.publish(angle_roll)
        self.pub_mira_pitch_joint_position.publish(angle_pitch)
        self.pub_mira_yaw_joint_position.publish(angle_yaw)

    def move_mira_roll_joint(self, position):
        """
        limits radians : lower="-0.2" upper="0.2"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_mira_roll_joint_position.publish(angle)

    def move_mira_pitch_joint(self, position):
        """
        limits radians : lower="0" upper="0.44"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_mira_pitch_joint_position.publish(angle)

    def move_mira_yaw_joint(self, position):
        """
        Limits : continuous, no limits
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_mira_yaw_joint_position.publish(angle)

    def mira_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.mira_joint_dictionary = dict(zip(msg.name, msg.position))

    def mira_check_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'pitch_joint', 'roll_joint', 'yaw_joint' is near the value given
        :param value:
        :return:
        """
        similar = self.mira_joint_dictionary.get(joint_name) >= (value - error ) and self.mira_joint_dictionary.get(joint_name) <= (value + error )

        return similar

    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def mira_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'pitch_joint', 'roll_joint', 'yaw_joint' is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param value:
        :return:
        """
        joint_reading = self.mira_joint_dictionary.get(joint_name)
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error

        return similar

    

    def mira_movement_look(self, roll, pitch, yaw):
        """
        Make Mira look down
        :return:
        """
        check_rate = 5.0
        position_roll = roll
        position_pitch = pitch
        position_yaw = yaw

        similar_roll = False
        similar_pitch = False
        similar_yaw = False
        rate = rospy.Rate(check_rate)
        while not (similar_roll and similar_pitch and similar_yaw):
            self.move_mira_all_joints(position_roll, position_pitch, position_yaw)
            similar_roll = self.mira_check_continuous_joint_value(joint_name="roll_joint", value=position_roll)
            similar_pitch = self.mira_check_continuous_joint_value(joint_name="pitch_joint", value=position_pitch)
            similar_yaw = self.mira_check_continuous_joint_value(joint_name="yaw_joint", value=position_yaw)
            rate.sleep()


    def search_for_blob_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Hearing Blobs Moving Mira...")
        rospy.spin()



if __name__ == "__main__":
    rospy.init_node('mira_move_head_node', anonymous=True)
    mira_jointmover_object = MiraBlobTracker()
    mira_jointmover_object.search_for_blob_loop()

