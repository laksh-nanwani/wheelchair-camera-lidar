#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os

curr_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

IMAGE_PATH = os.path.join(curr_path, "data", "rgb")
ALIGNED_DEPTH_PATH = os.path.join(curr_path, "data", "aligned_depth")
VISUAL_ODOM_PATH = os.path.join(curr_path, "data", "visual_odom")
ODOM_PATH = os.path.join(curr_path, "data", "odom")
DEPTH_PATH = os.path.join(curr_path, "data", "depth")

NUM = 0

bridge = CvBridge()

class my_class:
    def __init__(self):
        self.save_msgs_flag = False

        self.msgs_dict = {"image":np.zeros(1),
                        "aligned_depth":np.zeros(1),
                        "depth":np.zeros(1),
                        "odom":np.zeros(1),
                        # "visual_odom":np.zeros(1),        
                        }

        self.img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.aligned_depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.aligned_depth_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # self.visual_odom_sub = rospy.Subscriber("/rtabmap/odom", Odometry, self.visual_odom_callback)

    def save_msgs(self):
        if not self.save_msgs_flag:
            return
        
        global NUM
        path = os.path.join(IMAGE_PATH, f"img_{str(NUM)}.png")
        print(path)
        print(type(self.msgs_dict["image"]))
        cv2.imwrite(path, self.msgs_dict["image"])

        path = os.path.join(ALIGNED_DEPTH_PATH, f"img_{str(NUM)}.png")
        print(path)
        cv2.imwrite(path, self.msgs_dict["aligned_depth"])

        path = os.path.join(DEPTH_PATH, f"img_{str(NUM)}.png")
        print(path)
        cv2.imwrite(path, self.msgs_dict["depth"])

        path = os.path.join(ODOM_PATH, "pose_" + str(NUM))
        print(self.msgs_dict["odom"], path)
        np.save(path, self.msgs_dict["odom"])

        # path = os.path.join(VISUAL_ODOM_PATH, "pose_" + str(NUM))
        # print(self.msgs_dict["visual_odom"], path)
        # np.save(path, self.msgs_dict["visual_odom"])

        print(f"Saved {NUM}")
        NUM += 1
        self.save_msgs_flag = False

    def image_callback(self, msg):
        # global NUM
        # print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
        #     path = os.path.join(IMAGE_PATH, f"img_{str(NUM)}.png")
        #     print(path)
        #     cv2.imwrite(path, cv2_img)
            # print(type(cv2_img))
            self.msgs_dict["image"] = cv2_img

    def aligned_depth_callback(self, msg):
        # global NUM
        # print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            print(e)
        else:
        #     path = os.path.join(ALIGNED_DEPTH_PATH, f"img_{str(NUM)}.png")
        #     print(path)
        #     cv2.imwrite(path, cv2_img)
            self.msgs_dict["aligned_depth"] = cv2_img

    def depth_callback(self, msg):
        # global NUM
        # print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            print(e)
        else:
        #     path = os.path.join(DEPTH_PATH, f"img_{str(NUM)}.png")
        #     print(path)
        #     cv2.imwrite(path, cv2_img)
            self.msgs_dict["depth"] = cv2_img

    def visual_odom_callback(self, msg):
        # global NUM
        # print("Received odom!")
        # path = os.path.join(VISUAL_ODOM_PATH, "pose_" + str(NUM))
        # print(path)
        pose = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w])

        # print(pose, path)
        # np.save(path, pose)
        self.msgs_dict["visual_odom"] = pose
        # self.save_msgs_flag = True

    def odom_callback(self, msg):
        # global NUM
        # print("Received odom!")
        # path = os.path.join(ODOM_PATH, "pose_" + str(NUM))
        # print(path)
        pose = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w])

        # print(pose, path)
        # np.save(path, pose)
        self.msgs_dict["odom"] = pose
        self.save_msgs_flag = True


if __name__ == "__main__":
    rospy.init_node("Time_sync")
    my_obj = my_class()
    # publisher = rospy.Publisher("/lol_topic", Odometry, queue_size = 1)

    # rate = rospy.Rate(8)

    while not rospy.is_shutdown():
        my_obj.save_msgs()
    
    # rospy.spin()