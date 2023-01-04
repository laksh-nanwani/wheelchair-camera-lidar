#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

# PATH = "~/wheelchair-camera-lidar/workspace/src/vlmaps/vlmaps_data_creator/data/rgb/"
curr_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(curr_path, "data", "aligned_depth")
NUM = 0

bridge = CvBridge()

def callback(msg):
    global NUM
    print("Received an image!")
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "16UC1")
    except CvBridgeError as e:
        print(e)
    else:
        path = os.path.join(PATH, f"img_{str(NUM)}.png")
        print(path)
        cv2.imwrite(path, cv2_img)
        NUM += 1

def main():
    rospy.init_node('aligned_depth_saver_vlmaps')
    topic = "/camera/aligned_depth_to_color/image_raw"
    rospy.Subscriber(topic, Image, callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()