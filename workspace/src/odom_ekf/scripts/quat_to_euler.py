#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

class QuatToEuler:
    def __init__(self, msg_type):
        self.msg_type = msg_type

        # if msg_type == "imu":
        #     self.sub = rospy.Subscriber("/imu/data", Imu, self.convertQuatToEuler)
        # elif msg_type == "pose":
        #     self.sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.convertQuatToEuler)
        
        self.sub = rospy.Subscriber("/odom", Odometry, self.convertQuatToEuler)

        self.pub = rospy.Publisher("/orientation_string", String, queue_size = 10)

    def convertQuatToEuler(self, msg):
        # if self.msg_type == "imu":
        #     orientation_q = msg.orientation
        # elif self.msg_type == "pose":
        #     orientation_q = msg.pose.pose.orientation
        orientation_q = msg.pose.pose.orientation

        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = np.rad2deg(np.array(euler_from_quaternion(orientation_list)))

        x,y,z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z

        out_msg = f"Roll: {roll} Pitch: {pitch} Yaw: {yaw}"
        # out_msg = f"{x}, {y}, {z}"
        self.pub.publish(out_msg)

if __name__ == "__main__":
    rospy.init_node("quat_to_euler", anonymous=False)
    QuatToEuler(msg_type = "pose")
    rospy.spin()