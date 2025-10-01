#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from livox_ros_driver2.msg import CustomMsg


class PointCloudFilter:
    def __init__(self):
        rospy.init_node("pointcloud_angle_filter", anonymous=True)

        # Parameters
        self.input_topic = rospy.get_param("~input_topic", "/livox/lidar")
        self.output_topic = rospy.get_param("~output_topic", "/livox/lidar_fov")
        self.input_type = rospy.get_param("~input_type", "pointcloud2")  # "pointcloud2" or "livox"

        if self.input_type == "pointcloud2":
            self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
            rospy.Subscriber(self.input_topic, PointCloud2, self.cloud_callback)
            rospy.loginfo(f"[PointCloudFilter] Listening for PointCloud2 on {self.input_topic}")
        elif self.input_type == "livox":
            self.pub = rospy.Publisher(self.output_topic, CustomMsg, queue_size=1)
            rospy.Subscriber(self.input_topic, CustomMsg, self.livox_callback)
            rospy.loginfo(f"[PointCloudFilter] Listening for Livox CustomMsg on {self.input_topic}")
        else:
            rospy.logerr("Invalid ~input_type. Use 'pointcloud2' or 'livox'.")

    def cloud_callback(self, msg: PointCloud2):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        filtered_points = self.filter_points(points)
        filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered_points)
        self.pub.publish(filtered_msg)

    def livox_callback(self, msg: CustomMsg):
        filtered_msg = CustomMsg()
        filtered_msg.header = msg.header
        filtered_msg.timebase = msg.timebase
        filtered_msg.point_num = 0
        filtered_msg.lidar_id = msg.lidar_id
        filtered_msg.rsvd = msg.rsvd

        for p in msg.points:
            angle = math.degrees(math.atan2(p.y, p.x))
            if -135 <= angle <= 135:
                filtered_msg.points.append(p)
                filtered_msg.point_num += 1

        rospy.loginfo(f"{msg.point_num}, {filtered_msg.point_num}")

        self.pub.publish(filtered_msg)

    def filter_points(self, points):
        """Filter points based on -135° to +135° FOV"""
        return [(x, y, z) for x, y, z in points if -135 <= math.degrees(math.atan2(y, x)) <= 135]


if __name__ == "__main__":
    try:
        PointCloudFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
