#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os

# curr_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
path = "/home/laksh/vlmaps_data"
folder_name = "testing_m2f_p3dx"

IMAGE_PATH = os.path.join(path, folder_name, "rgb")
ALIGNED_DEPTH_PATH = os.path.join(path, folder_name, "aligned_depth")
ODOM_PATH = os.path.join(path, folder_name, "pose")
DEPTH_PATH = os.path.join(path, folder_name, "depth")

NUM = 0

bridge = CvBridge()

class my_class:
    def __init__(self):
        self.odom_sub = message_filters.Subscriber("/RosAria/pose", Odometry)
        self.img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.aligned_depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        # self.ts = message_filters.TimeSynchronizer([self.img_sub, self.aligned_depth_sub, self.depth_sub, self.visual_odom_sub, self.odom_sub], 10)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.img_sub, self.aligned_depth_sub], 10, slop=0.1)
        # print("1")
        self.ts.registerCallback(self.combined_callback)
        # print(2)

    def combined_callback(self, msg1, msg2, msg3):
        print("here")
        global NUM
        self.odom_callback(msg1)
        self.image_callback(msg2)
        self.aligned_depth_callback(msg3)
        NUM += 1

    def image_callback(self, msg):
        global NUM
        print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # cv2_img = cv2.rotate(cv2_img, cv2.ROTATE_180)
            path = os.path.join(IMAGE_PATH, f"{str(NUM)}.png")
            print(path)
            cv2.imwrite(path, cv2_img)

    def aligned_depth_callback(self, msg):
        global NUM
        print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            print(e)
        else:
            # cv2_img = cv2.rotate(cv2_img, cv2.ROTATE_180)

            path = os.path.join(ALIGNED_DEPTH_PATH, f"{str(NUM)}.png")
            print(path)
            cv2.imwrite(path, cv2_img)
            
            depth_array = np.array(cv2_img, dtype=np.float32)
            path = os.path.join(DEPTH_PATH, f"{str(NUM)}")
            np.save(path, depth_array)

    def odom_callback(self, msg):
        global NUM
        print("Received odom!")
        path = os.path.join(ODOM_PATH, str(NUM))
        print(path)
        pose = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w])

        # print(pose, path)
        np.save(path, pose)


if __name__ == "__main__":
    rospy.init_node("Time_sync")
    my_obj = my_class()

    rospy.spin()