#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as point_cloud2
from std_msgs.msg import Header
import tf2_ros, tf2_sensor_msgs
import numpy as np
# import pcl_ros, pcl


class Pointcloud_3D_to_2D:
    def __init__(self, target_frame="chassis", z_range=[-0.3, 1.5]):
        self.target_frame = target_frame
        self.z_range = z_range

        # self.T_z_zero = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.pub = rospy.Publisher("/livox/pcd_2d", PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber("/livox/pcd", PointCloud2, self.pointcloud_callback)

    def transform_pointcloud(self, pointcloud_msg: PointCloud2):
        try:
            # Wait for the transform between frames
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                pointcloud_msg.header.frame_id,  # Source frame
                rospy.Time(0),  # Get the latest available transform
                rospy.Duration(1.0),
            )  # Timeout

            # Transform the PointCloud2 message
            transformed_pc = tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, trans)
            # print(pointcloud_msg.header.frame_id, transformed_pc.header.frame_id)
            return transformed_pc

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform failed: {e}")
            return None

    def pointcloud_callback(self, pcd_msg: PointCloud2):
        transformed_msg = self.transform_pointcloud(pcd_msg)
        if transformed_msg:
            points = point_cloud2.read_points(transformed_msg, field_names=("x", "y", "z"), skip_nans=True)

            xyz_array = np.array(list(points))
            print(xyz_array.shape)

            # print(xyz_array[:, 2] <= self.z_range[1])
            mask = np.where(((xyz_array[:, 2] >= self.z_range[0]) & (xyz_array[:, 2] <= self.z_range[1])))

            xyz_array = xyz_array[mask]
            xyz_array[:, 2] = 0

            header = Header()
            header.frame_id = transformed_msg.header.frame_id
            out_pcd_msg = point_cloud2.create_cloud_xyz32(header, points=xyz_array)

            self.pub.publish(out_pcd_msg)

            # pcl_data = pcl_ros.pointcloud2_to_xyz_array(transformed_msg)
            # passthrough = pcl_data.make_passthrough_filter()

            # # Filter along the Z axis
            # passthrough.set_filter_field_name("z")
            # passthrough.set_filter_limits(self.z_range[0], self.z_range[1])

            # # Use the filter to filter the point cloud
            # pcl_data = passthrough.filter()

            # # Apply the transformation to the point cloud
            # transformed_cloud = pcl.transformPointCloud(pcl_data, self.transformation_matrix)

            # # Convert the transformed PCL PointCloud back to PointCloud2
            # transformed_pc_msg = pcl_ros.pointcloud2.xyz_array_to_pointcloud2(
            #     transformed_cloud, transformed_msg.header.stamp, transformed_msg.header.frame_id
            # )

            # self.pub.publish(transformed_pc_msg)


if __name__ == "__main__":
    rospy.init_node("Pointcloud_3D_to_2D")
    rospy.loginfo("Pointcloud 3D to 2D node started")
    Pointcloud_3D_to_2D()

    while not rospy.is_shutdown():
        rospy.spin()
