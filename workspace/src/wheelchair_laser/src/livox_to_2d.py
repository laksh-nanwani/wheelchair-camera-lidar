#!/usr/bin/env python3
"""
livox_to_2d_scan_and_pcd.py

Subscribe:  /livox/lidar           (livox_ros_driver2/CustomMsg)
Publish:    /livox/scan_2d         (sensor_msgs/LaserScan)
            /livox/pcd_2d          (sensor_msgs/PointCloud2)

Parameters (rosparam or launch args):
  ~input_topic           (string)  default: /livox/lidar
  ~pc2_topic             (string)  default: /livox/pcd_2d
  ~scan_topic            (string)  default: /livox/scan_2d
  ~target_frame          (string)  default: chassis
  ~livox_frame           (string)  default: livox_frame
  ~z_min                 (double)  default: -0.3
  ~z_max                 (double)  default: 1.5
  ~angle_min             (double)  default: -3.14159
  ~angle_max             (double)  default:  3.14159
  ~angle_increment_deg   (double)  default: 0.5    # degrees
  ~range_min             (double)  default: 0.05
  ~range_max             (double)  default: 30.0
  ~queue_size            (int)     default: 2
"""

import rospy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from livox_ros_driver2.msg import CustomMsg  # livox message type

# Predefine fields for PointCloud2 xyz
POINTFIELDS_XYZ = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


class LivoxTo2D:
    def __init__(self):
        # params
        self.input_topic = rospy.get_param("~input_topic", "/livox/lidar")
        self.pc2_topic = rospy.get_param("~pc2_topic", "/livox/pcd_2d")
        self.scan_topic = rospy.get_param("~scan_topic", "/livox/scan_2d")

        self.target_frame = rospy.get_param("~target_frame", "chassis")
        self.livox_frame = rospy.get_param("~livox_frame", "livox_frame")

        self.z_min = float(rospy.get_param("~z_min", -0.3))
        self.z_max = float(rospy.get_param("~z_max", 1.5))

        self.angle_min = float(rospy.get_param("~angle_min", -np.pi))
        self.angle_max = float(rospy.get_param("~angle_max", np.pi))
        angle_inc_deg = float(rospy.get_param("~angle_increment_deg", 0.5))
        self.angle_increment = np.deg2rad(angle_inc_deg)

        self.range_min = float(rospy.get_param("~range_min", 0.05))
        self.range_max = float(rospy.get_param("~range_max", 30.0))

        self.queue_size = int(rospy.get_param("~queue_size", 2))

        # derived
        self.num_bins = int(np.ceil((self.angle_max - self.angle_min) / self.angle_increment))
        if self.num_bins <= 0:
            rospy.logwarn("Computed num_bins <= 0. Resetting to 720 bins.")
            self.num_bins = 720
            self.angle_increment = (self.angle_max - self.angle_min) / self.num_bins

        rospy.loginfo(
            f"LivoxTo2D: angle_min={self.angle_min:.3f}, angle_max={self.angle_max:.3f}, "
            f"angle_inc={self.angle_increment:.5f} rad, bins={self.num_bins}"
        )

        # tf setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_transform = self._wait_and_get_transform(self.target_frame, self.livox_frame, timeout=5.0)

        # pubs/subs
        self.pc2_pub = rospy.Publisher(self.pc2_topic, PointCloud2, queue_size=1)
        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, CustomMsg, self.callback, queue_size=self.queue_size)

    def _wait_and_get_transform(self, target_frame, source_frame, timeout=5.0):
        """Try to get transform once at startup. Retry until success (or until rospy is shutdown)."""
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
                rospy.loginfo(f"Got static transform {source_frame} -> {target_frame}")
                return trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                if (rospy.Time.now() - start).to_sec() > timeout:
                    rospy.logwarn(f"Transform {source_frame}->{target_frame} not available yet; retrying... ({e})")
                    start = rospy.Time.now()  # reset timer to keep warning intermittent
                rospy.sleep(0.2)
        return None

    def custommsg_to_xyz_numpy(self, livox_msg):
        """
        Convert CustomMsg.points -> Nx3 numpy array
        Assumes each element in livox_msg.points has .x, .y, .z (float)
        """
        pts = livox_msg.points
        if len(pts) == 0:
            return np.zeros((0, 3), dtype=np.float32)

        # vectorized construction: list comprehension is faster than loop for attribute access here
        arr = np.fromiter((coord for p in pts for coord in (p.x, p.y, p.z)), dtype=np.float32, count=3 * len(pts))
        arr = arr.reshape(-1, 3)
        return arr

    def callback(self, livox_msg: CustomMsg):
        # convert to numpy xyz
        xyz = self.custommsg_to_xyz_numpy(livox_msg)
        if xyz.shape[0] == 0:
            return

        # Create a PointCloud2 in livox frame for transformation
        header = Header()
        header.stamp = livox_msg.header.stamp if hasattr(livox_msg, "header") else rospy.Time.now()
        header.frame_id = self.livox_frame
        raw_pc2 = pc2.create_cloud(header, POINTFIELDS_XYZ, xyz)

        # Transform cloud with cached transform (if available); otherwise try live transform
        if self.static_transform is not None:
            try:
                transformed_pc2 = do_transform_cloud(raw_pc2, self.static_transform)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"do_transform_cloud failed with cached transform: {e}")
                # try live transform fallback
                transformed_pc2 = self._do_live_transform(raw_pc2)
        else:
            transformed_pc2 = self._do_live_transform(raw_pc2)

        if transformed_pc2 is None:
            return

        # Read points back into numpy (x,y,z)
        points = np.array(
            list(pc2.read_points(transformed_pc2, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32
        )
        if points.size == 0:
            return

        # z-filter and project to 2D
        z_mask = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        points2d = points[z_mask][:, :2]  # Nx2 (x,y)
        if points2d.shape[0] == 0:
            # still publish an empty scan (all inf) and empty pc if you want
            self._publish_empty_scan(header.stamp)
            self._publish_pointcloud2([], header.stamp)
            return

        # compute ranges and angles
        xs = points2d[:, 0]
        ys = points2d[:, 1]
        ranges = np.hypot(xs, ys)
        angles = np.arctan2(ys, xs)

        # clamp ranges to [range_min, range_max]
        valid_range_mask = (ranges >= self.range_min) & (ranges <= self.range_max)
        ranges = ranges[valid_range_mask]
        angles = angles[valid_range_mask]

        if ranges.size == 0:
            self._publish_empty_scan(header.stamp)
            self._publish_pointcloud2([], header.stamp)
            return

        # Convert angles to bin indices
        # indices = floor((angle - angle_min) / angle_increment)
        idxs = np.floor((angles - self.angle_min) / self.angle_increment).astype(np.int32)

        # Keep only indices within [0, num_bins-1]
        valid_idx_mask = (idxs >= 0) & (idxs < self.num_bins)
        idxs = idxs[valid_idx_mask]
        ranges = ranges[valid_idx_mask]

        if ranges.size == 0:
            self._publish_empty_scan(header.stamp)
            self._publish_pointcloud2([], header.stamp)
            return

        # Build ranges array and take min per bin
        scan_ranges = np.full((self.num_bins,), np.inf, dtype=np.float32)
        # np.minimum.at will take elementwise min for each index
        np.minimum.at(scan_ranges, idxs, ranges)

        # Convert np.inf to float('inf') remains okay for LaserScan
        scan_msg = LaserScan()
        scan_msg.header.stamp = header.stamp
        scan_msg.header.frame_id = self.target_frame
        scan_msg.angle_min = float(self.angle_min)
        scan_msg.angle_max = float(self.angle_min + self.num_bins * self.angle_increment)
        scan_msg.angle_increment = float(self.angle_increment)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.0
        scan_msg.range_min = float(self.range_min)
        scan_msg.range_max = float(self.range_max)
        # convert to Python list (ROS expects list of floats)
        # replace np.inf with float('inf') explicitly (ROS can encode inf)
        scan_msg.ranges = [float(r) if np.isfinite(r) else float("inf") for r in scan_ranges.tolist()]

        # publish
        self.scan_pub.publish(scan_msg)

        # For 2D pointcloud publish with z=0
        pcd_points_2d = np.column_stack(
            (
                xs[valid_range_mask][valid_idx_mask],  # careful indexing: xs filtered earlier
                ys[valid_range_mask][valid_idx_mask],
                np.zeros(len(idxs), dtype=np.float32),
            )
        )
        # The above line uses matching masks; alternatively simpler to rebuild from angles & ranges:
        # reconstruct x,y from angles and ranges for published pc:
        x_pub = ranges * np.cos(angles)
        y_pub = ranges * np.sin(angles)
        pcd_points_2d = np.column_stack((x_pub, y_pub, np.zeros_like(x_pub, dtype=np.float32)))

        self._publish_pointcloud2(pcd_points_2d, header.stamp)

    def _do_live_transform(self, pc2_msg):
        try:
            live_trans = self.tf_buffer.lookup_transform(
                self.target_frame, pc2_msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1)
            )
            return do_transform_cloud(pc2_msg, live_trans)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Live transform failed: {e}")
            return None

    def _publish_pointcloud2(self, xyz_array, stamp):
        # xyz_array: Nx3 numpy or list
        if isinstance(xyz_array, np.ndarray) and xyz_array.size > 0:
            pts = xyz_array.astype(np.float32).tolist()
        else:
            pts = []
        header = Header()
        header.stamp = stamp if stamp is not None else rospy.Time.now()
        header.frame_id = self.target_frame
        out_pc2 = pc2.create_cloud(header, POINTFIELDS_XYZ, pts)
        self.pc2_pub.publish(out_pc2)

    def _publish_empty_scan(self, stamp):
        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp if stamp is not None else rospy.Time.now()
        scan_msg.header.frame_id = self.target_frame
        scan_msg.angle_min = float(self.angle_min)
        scan_msg.angle_max = float(self.angle_min + self.num_bins * self.angle_increment)
        scan_msg.angle_increment = float(self.angle_increment)
        scan_msg.range_min = float(self.range_min)
        scan_msg.range_max = float(self.range_max)
        scan_msg.ranges = [float("inf")] * self.num_bins
        self.scan_pub.publish(scan_msg)


if __name__ == "__main__":
    rospy.init_node("livox_to_2d_scan_and_pcd", anonymous=True)
    node = LivoxTo2D()
    rospy.loginfo("livox_to_2d_scan_and_pcd started")
    rospy.spin()
