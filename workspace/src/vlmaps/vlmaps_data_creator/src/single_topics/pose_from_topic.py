#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import os

curr_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# PATH = "~/wheelchair-camera-lidar/workspace/src/vlmaps/vlmaps_data_creator/data/odom/"
PATH = os.path.join(curr_path, "data", "odom")
NUM = 0

def callback(msg):
    global NUM
    print("Received odom!")
    path = os.path.join(PATH, "pose_" + str(NUM))
    pose = np.array([msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w])

    print(pose, path)

    np.save(path, pose)
    NUM += 1

def main():
    rospy.init_node('odom_saver_vlmaps')
    topic = "/odom"
    print(PATH)
    if not os.path.exists(PATH):
        print(False)
        os.makedirs(PATH)
    rospy.Subscriber(topic, Odometry, callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()