#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as point_cloud2
from std_msgs.msg import Header
import tf2_ros, tf2_sensor_msgs
import numpy as np
# from pcdarray import PC2_Array

# import pcl_ros, pcl
import numpy as np
from sensor_msgs.msg import PointField

__datatypes = [
    np.dtype("int8"),
    np.dtype("uint8"),
    np.dtype("int16"),
    np.dtype("uint16"),
    np.dtype("int32"),
    np.dtype("uint32"),
    np.dtype("float32"),
    np.dtype("float64"),
]

__type_dict = {getattr(PointField, dtype.name.upper()): dtype for dtype in __datatypes}

__type_size = {ftype: np.dtype(dtype).itemsize for ftype, dtype in __type_dict.items()}


def __get_X3d(array):
    points = np.zeros((array.shape[0], 3), dtype=np.float32)
    points[:, 0] = array["x"]
    points[:, 1] = array["y"]
    points[:, 2] = array["z"]
    return points


def PC2_Array(pc2_msg: PointCloud2):
    # dtypes = []
    # offset = 0
    # for field in pc2_msg.fields:
    #     """Each field contains (name, offset, datatype, count) attribs"""
    #     dtype = __type_dict[field.datatype]
    #     dtypes.append((field.name, dtype))
    #     print(field.datatype, __type_size[field.datatype])
    #     offset += __type_size[field.datatype]
    #     # print(field)

    # print(dtypes, len(pc2_msg.data), len(dtypes))
    dtypes = [
        ("x", np.dtype("float32")),
        ("y", np.dtype("float32")),
        ("z", np.dtype("float32")),
        ("_", np.dtype("float32")),
        ("intensity", np.dtype("float32")),
        # ("ring", np.dtype("uint16")),
    ]

    arr = np.frombuffer(pc2_msg.data, dtypes)
    print((arr[0]))
    arr = arr.reshape((pc2_msg.width,))
    return __get_X3d(arr)


class Pointcloud_3D_to_2D:
    def __init__(self, target_frame="base_link", z_range=[-0.3, 1.5]):
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
        self.sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.pointcloud_callback)

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
            # points = point_cloud2.read_points(transformed_msg, field_names=("x", "y", "z"), skip_nans=True)
            xyz_array = PC2_Array(transformed_msg)
            print(xyz_array.shape)

            # xyz_array = np.array(list(points))
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
