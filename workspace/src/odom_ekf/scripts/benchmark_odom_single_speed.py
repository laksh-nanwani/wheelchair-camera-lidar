#!/usr/bin/env python

from operator import mod
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import sys
from tf.transformations import euler_from_quaternion
'''
Lin vel, dist: inc
measure: X, Y, yaw, drive, turn, compass change
'''
MAX_LIN_VEL = 0.3 # 0.05, 0.1, 0.2, 0.3
MAX_ANG_VEL = 0.2
DIST = 4 # 1, 2, 3, 4

class Benchmark:
    def __init__(self, dist = 0, angle = 0, mode = 'd'):  # mode: 'd' / 'a'
        # self.linear_vel = linear
        # self.angular_vel = angular
        self.dist = dist
        self.angle = angle


        self.twist = Twist()
        self.mode = mode
        
        self.pub = rospy.Publisher("/wheelchair_diff/cmd_vel", Twist, queue_size = 10)

        if mode == 'd':
            if dist == 0:
                sys.exit()
            if dist > 0:
                self.sign = 1
            else:
                self.sign = -1

            self.twist.linear.x = self.sign * MAX_LIN_VEL
            self.sub = rospy.Subscriber("/odom", Odometry, self.check_distance)

        elif mode == 'a':
            if angle > 0 and angle <= 180:
                self.sign = 1
            elif angle < 0 and angle >= -180:
                self.sign = -1
            else:
                rospy.signal_shutdown("Goal Reached")

            self.twist.angular.z = self.sign * MAX_ANG_VEL
            self.sub = rospy.Subscriber("/odom", Odometry, self.check_angle)

    def check_distance(self, msg):
        if self.sign * (msg.pose.pose.position.x**2 + msg.pose.pose.position.y**2)**0.5 < self.sign * self.dist:
            self.pub.publish(self.twist)
            rospy.loginfo(f"Given distance:{self.dist}")
            rospy.loginfo(f"x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}")
        else:
            self.twist.linear.x = 0
            self.pub.publish(self.twist)
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (_, _, yaw) = np.rad2deg(np.array(euler_from_quaternion(orientation_list)))

            rospy.sleep(2)
            rospy.loginfo(f"Given distance:{self.dist}")
            rospy.loginfo(f"x: {msg.pose.pose.position.x} , y: {msg.pose.pose.position.y} , Yaw: {yaw}")
            rospy.signal_shutdown("Goal Reached")
            

    def check_angle(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = np.rad2deg(np.array(euler_from_quaternion(orientation_list)))

        if self.sign * yaw < self.sign * self.angle:
            self.pub.publish(self.twist)
        else:
            self.twist.linear.x = 0
            self.pub.publish(self.twist)

            rospy.loginfo(f"Given Angle:{self.angle}")
            rospy.loginfo(f"{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {yaw}")
            rospy.signal_shutdown("Goal Reached")



if __name__ == "__main__":
    rospy.init_node("Odom_Benchmark", anonymous=False)
    Benchmark(dist = DIST)
    rospy.loginfo("Odom Benchmarking Started")
    rospy.spin()
