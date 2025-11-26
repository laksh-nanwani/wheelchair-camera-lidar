# ...existing code...
import os
import math
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion


class ProbabilityGridMap(object):
    def __init__(self):
        # params (use ROS params or defaults)
        self.resolution = rospy.get_param("~map_resolution", 0.05)
        self.width = int(rospy.get_param("~map_width", 800))
        self.height = int(rospy.get_param("~map_height", 800))
        self.origin_x = float(rospy.get_param("~map_origin_x", -20.0))
        self.origin_y = float(rospy.get_param("~map_origin_y", -20.0))
        self.z_min = float(rospy.get_param("~z_min", -1.0))
        self.z_max = float(rospy.get_param("~z_max", 1.0))
        self.z_flat = float(rospy.get_param("~z_flat", 0.0))
        self.save_path = rospy.get_param("~save_path", "/tmp/occupancy_map.npy")
        self.l_free = float(rospy.get_param("~logodd_free", -0.4))
        self.l_occ = float(rospy.get_param("~logodd_occ", 0.85))
        self.l_min = float(rospy.get_param("~logodd_min", -5.0))
        self.l_max = float(rospy.get_param("~logodd_max", 5.0))
        self.publish_rate = float(rospy.get_param("~publish_rate_hz", 5.0))

        sensor_x = rospy.get_param("~sensor_x", None)
        sensor_y = rospy.get_param("~sensor_y", None)
        if sensor_x is None:
            sensor_x = self.origin_x + (self.width * self.resolution) / 2.0
        if sensor_y is None:
            sensor_y = self.origin_y + (self.height * self.resolution) / 2.0
        self.sensor_x = float(sensor_x)
        self.sensor_y = float(sensor_y)

        # log-odds grid (row-major [y,x]); index as [row, col] -> [y,x]
        self.logodds = np.zeros((self.height, self.width), dtype=np.float32)

        # ROS interfaces
        self.pc_sub = rospy.Subscriber("/points", PointCloud2, self.pc_callback, queue_size=1)
        self.grid_pub = rospy.Publisher("/occupancy_grid", OccupancyGrid, queue_size=1)

        # timer to publish grid periodically
        rospy.Timer(rospy.Duration(1.0 / max(0.1, self.publish_rate)), lambda ev: self.publish_grid())

        rospy.loginfo("ProbabilityGridMap (ROS1) started")

    def world_to_cell(self, x, y):
        cx = int(math.floor((x - self.origin_x) / self.resolution))
        cy = int(math.floor((y - self.origin_y) / self.resolution))
        return cx, cy

    def in_bounds(self, cx, cy):
        return 0 <= cx < self.width and 0 <= cy < self.height

    def bresenham(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        if dy <= dx:
            err = dx / 2.0
            while x != x1:
                yield x, y
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
            yield x, y
        else:
            err = dy / 2.0
            while y != y1:
                yield x, y
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
            yield x, y

    def pc_callback(self, msg):
        pts = []
        try:
            for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                pts.append(p)
        except Exception:
            for p in point_cloud2.read_points(msg, skip_nans=True):
                pts.append((p[0], p[1], p[2]))

        if not pts:
            return

        pts = np.array(pts, dtype=np.float32)

        # filter by z
        mask = (pts[:, 2] <= self.z_max) & (pts[:, 2] >= self.z_min)
        pts = pts[mask]
        if pts.shape[0] == 0:
            return

        # flatten points to plane
        pts[:, 2] = self.z_flat

        sx_cell, sy_cell = self.world_to_cell(self.sensor_x, self.sensor_y)
        if not self.in_bounds(sx_cell, sy_cell):
            rospy.logwarn("Sensor origin outside map bounds, skipping update")
            return

        for x, y, _z in pts:
            cx, cy = self.world_to_cell(float(x), float(y))
            if not self.in_bounds(cx, cy):
                continue
            for rx, ry in self.bresenham(sx_cell, sy_cell, cx, cy):
                if not self.in_bounds(rx, ry):
                    break
                if rx == cx and ry == cy:
                    self.logodds[ry, rx] += self.l_occ
                else:
                    self.logodds[ry, rx] += self.l_free

                # clamp
                if self.logodds[ry, rx] > self.l_max:
                    self.logodds[ry, rx] = self.l_max
                if self.logodds[ry, rx] < self.l_min:
                    self.logodds[ry, rx] = self.l_min

    def publish_grid(self):
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"

        meta = MapMetaData()
        meta.map_load_time = rospy.Time.now()
        meta.resolution = self.resolution
        meta.width = self.width
        meta.height = self.height
        origin = Pose()
        origin.position = Point(self.origin_x, self.origin_y, 0.0)
        origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        meta.origin = origin
        grid.info = meta

        probs = 1.0 / (1.0 + np.exp(-self.logodds))
        eps = 1e-3
        flat = probs.flatten(order="C")  # row-major
        data = []
        for p in flat:
            if abs(p - 0.5) < eps:
                data.append(-1)
            else:
                data.append(int(np.clip(np.round(p * 100.0), 0, 100)))
        grid.data = data
        try:
            self.grid_pub.publish(grid)
        except rospy.ROSException:
            pass

    def save_map(self, path=None):
        if path is None:
            path = self.save_path
        try:
            dirname = os.path.dirname(path)
            if dirname and not os.path.exists(dirname):
                os.makedirs(dirname, exist_ok=True)
            np.save(path, self.logodds)
            probs = 1.0 / (1.0 + np.exp(-self.logodds))
            img = (probs * 255.0).astype(np.uint8)
            try:
                import cv2

                cv2.imwrite(path + ".png", np.flipud(img))
            except Exception:
                with open(path + ".pgm", "wb") as f:
                    header = f"P5 {self.width} {self.height} 255\n".encode("ascii")
                    f.write(header)
                    f.write(np.flipud(img).tobytes())
            rospy.loginfo("Saved map to %s (and PNG/PGM alongside)" % path)
        except Exception as e:
            rospy.logerr("Failed to save map: %s" % str(e))


def main():
    rospy.init_node("probability_grid_map", anonymous=False)
    pgm = ProbabilityGridMap()
    rospy.on_shutdown(lambda: pgm.save_map())
    rospy.spin()


if __name__ == "__main__":
    main()
# ...existing code...
