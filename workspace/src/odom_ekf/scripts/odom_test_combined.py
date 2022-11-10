#!/usr/bin/env python

from operator import mod
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
'''
Lin vel, dist: inc
measure: X, Y, yaw, drive, turn, compass change
'''
MAX_LIN_VEL = 0.01
MAX_ANG_VEL = 0.25
DIST = 2
ANGLE = 180

class Benchmark:
    def __init__(self, mode = 0):
        self.twist = Twist()
        self.mode = mode
        
        self.initial_run = True
        self.pub = rospy.Publisher("/wheelchair_diff/cmd_vel", Twist, queue_size = 10)

        self.sub = rospy.Subscriber("/odom", Odometry, self.check_odom)

    def check_odom(self, msg):
        if self.initial_run:
            yaw = self.get_yaw(msg)
            self.last_state_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
            self.initial_run = False

        if self.mode == 0:
            self.check_distance(msg)
        elif self.mode == 1:
            self.check_angle(msg)
        elif self.mode == 2:
            self.check_distance(msg)
        

    def check_distance(self, msg):
        if ((msg.pose.pose.position.x - self.last_state_pose[0])**2 + 
            (msg.pose.pose.position.y - self.last_state_pose[1])**2)**0.5 < DIST:
            
            self.twist.linear.x = MAX_LIN_VEL
            self.pub.publish(self.twist)
            rospy.loginfo(f"Given distance:{DIST}")
            rospy.loginfo(f"x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}")
        else:
            self.twist.linear.x = 0
            self.pub.publish(self.twist)
            yaw = self.get_yaw(msg)

            self.last_state_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
            rospy.sleep(2)
            rospy.loginfo(f"Given distance:{DIST}")
            rospy.loginfo(f"x: {self.last_state_pose[0]} , y: {self.last_state_pose[1]} , Yaw: {yaw}")

            if self.mode == 0:
                self.mode = 1
                
            else:
                rospy.signal_shutdown("Goal Reached")
            

    def check_angle(self, msg):
        yaw = self.get_yaw(msg)

        if (self.last_state_pose[2] - yaw) < ANGLE:
            self.twist.angular.z = MAX_ANG_VEL
            self.pub.publish(self.twist)
        else:
            self.twist.angular.z = 0
            self.pub.publish(self.twist)

            self.last_state_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
            rospy.sleep(2)
            rospy.loginfo(f"Given angle:{ANGLE}")
            rospy.loginfo(f"x: {self.last_state_pose[0]} , y: {self.last_state_pose[1]} , Yaw: {yaw}")

            self.mode = 2

    def get_yaw(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = np.rad2deg(np.array(euler_from_quaternion(orientation_list)))
        if yaw < 0:
            return 360 + yaw
        return yaw

if __name__ == "__main__":
    rospy.init_node("Odom_Benchmark", anonymous=False)
    Benchmark()
    rospy.loginfo("Odom Benchmarking Started")
    rospy.spin()
