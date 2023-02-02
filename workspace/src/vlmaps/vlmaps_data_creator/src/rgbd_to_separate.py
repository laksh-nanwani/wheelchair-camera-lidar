#!/usr/bin/env python3
import rospy
from rtabmap_ros.msg import RGBDImage
# from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os

# curr_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# path = os.path.join(curr_path, "data")
path = "/home/laksh/vlmaps_data"
folder_name = "data_p3dx_o3d"

IMAGE_PATH = os.path.join(path, folder_name, "rgb")
DEPTH_PATH = os.path.join(path, folder_name, "depth")

NUM = 0
FACTOR = 1

bridge = CvBridge()

class my_class:
    def __init__(self):
        self.rgbd_sub = rospy.Subscriber("/camera/rgbd_image", RGBDImage, self.save_msgs)

    def save_msgs(self, msg):
        global NUM
        if NUM >= 0:
            try:
                rgb = bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
                depth = bridge.imgmsg_to_cv2(msg.depth, desired_encoding = "passthrough")
            except CvBridgeError as e:
                print(e)
            else:
                rgb = cv2.rotate(rgb, cv2.ROTATE_180)
                depth = cv2.rotate(depth, cv2.ROTATE_180)
                rgb_path = os.path.join(IMAGE_PATH, f"{str(NUM)}.jpg")
                depth_path = os.path.join(DEPTH_PATH, f"{str(NUM)}.png")
                print(rgb_path)
                cv2.imwrite(rgb_path, rgb)
                cv2.imwrite(depth_path, depth)
                NUM += 1

if __name__ == "__main__":
    rospy.init_node("RGBD_image_separator")
    rospy.loginfo("RGBD_image_separator started")
    my_obj = my_class()
    rospy.spin()