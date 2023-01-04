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
        self.odom_sub = message_filters.Subscriber("/odom", Odometry)
        self.visual_odom_sub = message_filters.Subscriber("/rtabmap/odom", Odometry)
        self.img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        self.aligned_depth_sub = message_filters.Subscriber("/camera/aligned_depth/image_raw", Image)
        self.ts = message_filters.TimeSynchronizer([self.img_sub, self.aligned_depth_sub, self.depth_sub, self.visual_odom_sub, self.odom_sub], 10)
        self.ts.registerCallback(self.combined_callback)

    def combined_callback(self, msg1, msg2, msg3, msg4, msg5):
        print("here")
        global NUM
        self.image_callback(msg1)
        self.aligned_depth_callback(msg2)
        self.depth_callback(msg3)
        self.visual_odom_callback(msg4)
        self.odom_callback(msg5)
        NUM += 1


    def image_callback(self, msg):
        global NUM
        print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            path = os.path.join(IMAGE_PATH, f"img_{str(NUM)}.png")
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
            path = os.path.join(ALIGNED_DEPTH_PATH, f"img_{str(NUM)}.png")
            print(path)
            cv2.imwrite(path, cv2_img)

    def depth_callback(self, msg):
        global NUM
        print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            print(e)
        else:
            path = os.path.join(DEPTH_PATH, f"img_{str(NUM)}.png")
            print(path)
            cv2.imwrite(path, cv2_img)

    def visual_odom_callback(self, msg):
        global NUM
        print("Received odom!")
        path = os.path.join(VISUAL_ODOM_PATH, "pose_" + str(NUM))
        print(path)
        pose = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w])

        print(pose, path)

        np.save(path, pose)

    def odom_callback(self, msg):
        global NUM
        print("Received odom!")
        path = os.path.join(ODOM_PATH, "pose_" + str(NUM))
        print(path)
        pose = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w])

        print(pose, path)

        np.save(path, pose)


if __name__ == "__main__":
    rospy.init_node("Time_sync")
    my_obj = my_class()
    # publisher = rospy.Publisher("/lol_topic", Odometry, queue_size = 1)

    # rate = rospy.Rate(8)

    # while not rospy.is_shutdown():
    #         publisher.publish(my_obj.odom)
    #         rate.sleep()

    rospy.spin()