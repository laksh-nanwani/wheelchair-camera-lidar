#!/usr/bin/env python3
"""
conical_mirror_reflect.py

ROS1 node that:
 - subscribes to a PointCloud2 (LiDAR) in LiDAR frame (origin at optical center)
 - for all points with z > 0, calculates their true 3D position after
   reflecting off the conical mirror and colors them RED.
 - publishes modified PointCloud2 with color on 'mirror/points'
 - publishes a Marker for the truncated cone for RViz debugging.
"""

import rospy
import math
import struct
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA


# Helper function to pack RGB values into a single float
def pack_color(r, g, b):
    """Packs RGB into a single float for PointCloud2."""
    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
    # Pack the 32-bit integer into bytes, then unpack as a 32-bit float.
    packed = struct.pack("I", rgb_int)
    return struct.unpack("f", packed)[0]


class ConicalMirrorReflector:
    def __init__(self):
        rospy.init_node("conical_mirror_reflector", anonymous=True)

        # params
        self.in_topic = rospy.get_param("~input_topic", "/livox/lidar")
        self.out_topic = rospy.get_param("~output_topic", "/mirror/points")
        self.alpha_deg = rospy.get_param("~mirror_angle_deg", 27.5)
        self.r0 = rospy.get_param("~mirror_base_radius", 0.1)
        self.cone_dir = rospy.get_param("~cone_direction", "down")
        self.max_dist = rospy.get_param("~max_intersect_dist", 50.0)

        self.alpha = math.radians(self.alpha_deg)

        self.k = math.tan(self.alpha)
        if self.cone_dir == "down":
            self.k = -abs(self.k)
        else:
            self.k = abs(self.k)

        self.pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher("~/mirror_marker", Marker, queue_size=1, latch=True)
        rospy.Subscriber(self.in_topic, PointCloud2, self.cloud_cb, queue_size=1)

        rospy.loginfo("ConicalMirrorReflector: Reflecting all points with z > 0.")
        rospy.loginfo("Params: alpha=%.3f deg, r0=%.3fm, cone_dir=%s", self.alpha_deg, self.r0, self.cone_dir)

        # Pre-pack colors for efficiency
        self.color_red = pack_color(255, 0, 0)
        self.color_white = pack_color(200, 200, 200)  # Use gray for direct points

        rospy.sleep(0.2)
        self.publish_cone_marker()

    def publish_cone_marker(self):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = rospy.Time.now()
        m.ns = "conical_mirror"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        vis_height = 0.1
        top_r = self.r0 + vis_height * abs(math.tan(self.alpha))
        m.scale.x = 2.0 * top_r
        m.scale.y = 2.0 * top_r
        m.scale.z = vis_height
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = vis_height / 2.0
        m.pose.orientation.w = 1.0
        m.color = ColorRGBA(0.2, 0.6, 1.0, 0.3)
        self.marker_pub.publish(m)
        rospy.loginfo("Published cone marker (approx).")

    def intersect_cone(self, d):
        dx, dy, dz = d[0], d[1], d[2]
        s = math.hypot(dx, dy)
        denom = dz - s * self.k
        if abs(denom) < 1e-12:
            return None
        t = (-self.r0 * self.k) / denom
        if t <= 1e-6 or t > self.max_dist:
            return None
        return t * d

    def local_normal(self, p):
        x, y, _ = p
        r = math.hypot(x, y)
        if r < 1e-9:
            return np.array([0.0, 0.0, 1.0]) if self.k > 0 else np.array([0.0, 0.0, -1.0])
        nx = -self.k * (x / r)
        ny = -self.k * (y / r)
        nz = 1.0
        n = np.array([nx, ny, nz])
        return n / np.linalg.norm(n)

    def reflect_vec(self, incoming_dir, normal):
        return incoming_dir - 2.0 * np.dot(incoming_dir, normal) * normal

    def cloud_cb(self, msg):
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        out_pts_with_color = []

        for x, y, z in pts:
            if z > -0.05:
                # This ray is a reflection. Calculate its new position and color it red.
                total_dist = math.sqrt(x * x + y * y + z * z)
                if total_dist < 1e-6:
                    continue

                d = np.array([x / total_dist, y / total_dist, z / total_dist])
                p_int = self.intersect_cone(d)
                if p_int is None:
                    continue

                dist_to_mirror = np.linalg.norm(p_int)
                dist_remaining = total_dist - dist_to_mirror
                if dist_remaining < 0:
                    continue

                n = self.local_normal(p_int)
                d_ref = self.reflect_vec(d, n)
                p_actual = p_int + d_ref * dist_remaining

                # Append the new point with red color
                pt_data = [float(p_actual[0]), float(p_actual[1]), float(p_actual[2]), self.color_red]
                out_pts_with_color.append(pt_data)
            else:
                # This is a direct-view point. Color it white/gray.
                pt_data = [float(x), float(y), float(z), self.color_white]
                out_pts_with_color.append(pt_data)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id

        # MODIFICATION: Add the 'rgb' field to the point cloud definition
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]

        pc2_msg = pc2.create_cloud(header, fields, out_pts_with_color)
        self.pub.publish(pc2_msg)


if __name__ == "__main__":
    try:
        node = ConicalMirrorReflector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
