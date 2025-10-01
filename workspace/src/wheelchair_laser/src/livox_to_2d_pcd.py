#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs  # needed to use transform in numpy
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from livox_ros_driver2.msg import CustomMsg  # assuming you have this msg


class Pointcloud3DTo2D:
    def __init__(self, target_frame="chassis", z_range=(-0.3, 1.5)):
        self.target_frame = target_frame
        self.z_range = z_range

        # --- TF Buffer + One-Time Transform ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Waiting for transform between livox_frame and %s...", self.target_frame)
        self.static_transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            "livox",  # <-- CHANGE this to your actual LiDAR frame
            rospy.Time(0),
            rospy.Duration(2.0),
        )
        rospy.loginfo("Transform acquired. Using static transform for all pointclouds.")

        # Publisher
        self.pub = rospy.Publisher("/livox/pcd_2d", PointCloud2, queue_size=1)

        # Subscriber
        self.sub = rospy.Subscriber("/livox/lidar", CustomMsg, self.custom_msg_callback, queue_size=1)

        # Predefine fields for PointCloud2
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    def custom_msg_callback(self, livox_msg: CustomMsg):
        # --- Convert Livox CustomMsg to XYZ numpy ---
        # Efficient vectorized conversion
        xyz_array = np.array([[p.x, p.y, p.z] for p in livox_msg.points], dtype=np.float32)

        # --- Transform cloud once, using pre-fetched TF ---
        # Convert to PointCloud2
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "livox_frame"  # Original frame

        raw_pcd = pc2.create_cloud(header, self.fields, xyz_array)

        transformed_pc = do_transform_cloud(raw_pcd, self.static_transform)

        # Read points back (still in numpy, efficient)
        points = np.array(list(pc2.read_points(transformed_pc, field_names=("x", "y", "z"), skip_nans=True)))
        if points.size == 0:
            return

        # --- Z filtering + projection to 2D ---
        mask = (points[:, 2] >= self.z_range[0]) & (points[:, 2] <= self.z_range[1])
        points = points[mask]
        points[:, 2] = 0.0

        # --- Publish filtered PointCloud2 ---
        out_header = Header()
        out_header.stamp = rospy.Time.now()
        out_header.frame_id = self.target_frame
        out_pcd_msg = pc2.create_cloud_xyz32(out_header, points)
        self.pub.publish(out_pcd_msg)


if __name__ == "__main__":
    rospy.init_node("pointcloud_3d_to_2d", anonymous=True)
    rospy.loginfo("Pointcloud 3D→2D node started (using static TF)")
    pc_node = Pointcloud3DTo2D()
    rospy.spin()
