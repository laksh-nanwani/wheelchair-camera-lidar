#!/usr/bin/env python3

import numpy as np
import os
import time

# --- ROS DEPENDENCIES (REQUIRED) ---
import rospy
from sensor_msgs.msg import PointCloud2

# ROS message types for the output map and robot pose
from nav_msgs.msg import OccupancyGrid, Odometry  # ADDED Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

# The ROS utility for reading PointCloud2 data into a NumPy array
import sensor_msgs.point_cloud2 as pc2

# Assuming this custom message definition is available in your workspace for Livox:
from livox_ros_driver2.msg import CustomMsg

# Required for coordinate transformation from quaternion to rotation matrix
# You must have the ROS dependency 'tf' or 'tf2_ros' installed.
from tf.transformations import quaternion_matrix

# --- Configuration Constants ---
# Bounding Box Filters (applied locally in the sensor frame)
Z_UPPER_LIMIT = 0.1
Z_LOWER_LIMIT = -1.0
X_UPPER_LIMIT = 30.0
X_LOWER_LIMIT = -30.0
Y_UPPER_LIMIT = 30.0
Y_LOWER_LIMIT = -30.0

GRID_RESOLUTION = 0.05
INPUT_SOURCE_TYPE = "LIVOX_CUSTOM_MSG"
FINAL_SAVE_PATH = "occupancy_grid_final_map.npy"

# ROS Topic Configuration
MAP_TOPIC = "/occupancy_grid_map"
ODOM_TOPIC = "/Odometry"  # Topic for Odometry messages

# Fixed Global Map Bounds (Crucial for a persistent global grid)
GLOBAL_X_MIN = -50.0  # Global map boundary X minimum
GLOBAL_X_MAX = 50.0  # Global map boundary X maximum
GLOBAL_Y_MIN = -50.0  # Global map boundary Y minimum
GLOBAL_Y_MAX = 50.0  # Global map boundary Y maximum
GLOBAL_MAP_FRAME = "camera_init"  # Frame of the published map (Odometry parent frame)


# --- Helper Functions for Message Conversion (Actual ROS Logic) ---


def convert_pointcloud2_to_numpy(msg):
    """
    Converts a PointCloud2 message to an (N, 3) NumPy array (X, Y, Z).
    """
    points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pcd_array = np.array(list(points_gen), dtype=np.float32)

    if pcd_array.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    return pcd_array


def convert_livox_custom_to_numpy(msg):
    """
    Converts a Livox CustomMsg to an (N, 3) NumPy array (X, Y, Z).
    """
    # Use a list comprehension to pull out X, Y, Z coordinates
    pcd_list = [(p.x, p.y, p.z) for p in msg.points]
    pcd_array = np.array(pcd_list, dtype=np.float32)

    return pcd_array


# --- NEW: Transformation Helper ---


def pose_to_transform_matrix(pose):
    """
    Converts a geometry_msgs/Pose to a 4x4 homogeneous transformation matrix.
    Assumes the pose is T_odom_base.
    """
    # 1. Extract quaternion from Pose
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    # 2. Convert quaternion to 4x4 rotation matrix
    R = quaternion_matrix(q)

    # 3. Apply translation
    R[0, 3] = pose.position.x
    R[1, 3] = pose.position.y
    R[2, 3] = pose.position.z

    return R


# --- PCD Processor Class ---


class PCDProcessor:
    """
    A class to handle LiDAR Point Cloud processing and maintaining a persistent
    global occupancy grid based on Odometry-transformed scans.
    """

    def __init__(self, grid_resolution=0.05, log_odds_hit=1.0, initial_log_odds=0.0):
        """
        Initializes the processor with grid parameters and global map bounds.
        """
        self.resolution = grid_resolution
        self.log_odds_hit = log_odds_hit
        self.initial_log_odds = initial_log_odds

        # Fixed Global Map Parameters
        self.map_x_min = GLOBAL_X_MIN
        self.map_x_max = GLOBAL_X_MAX
        self.map_y_min = GLOBAL_Y_MIN
        self.map_y_max = GLOBAL_Y_MAX

        # Calculate fixed global grid dimensions
        map_width = self.map_x_max - self.map_x_min
        map_height = self.map_y_max - self.map_y_min
        self.cols = int(np.ceil(map_width / self.resolution))
        self.rows = int(np.ceil(map_height / self.resolution))

        # Global, persistent log-odds grid (Initialization)
        self.global_occupancy_grid_log = np.full((self.rows, self.cols), self.initial_log_odds, dtype=np.float32)

        # Storage for last processed data and pose
        self.last_robot_pose = None  # Stores the Pose message from Odometry

    def set_robot_pose(self, pose):
        """Stores the most recent robot pose."""
        self.last_robot_pose = pose

    # ... (generate_sample_pcd remains the same)

    def filter_pcd_by_xyz(self, pcd, x_min, x_max, y_min, y_max, z_min, z_max):
        """
        Filters the Point Cloud to include only points within the specified
        X, Y, and Z bounding box (in sensor frame).
        """
        if pcd.size == 0:
            return pcd

        x_mask = (pcd[:, 0] >= x_min) & (pcd[:, 0] <= x_max)
        y_mask = (pcd[:, 1] >= y_min) & (pcd[:, 1] <= y_max)
        z_mask = (pcd[:, 2] >= z_min) & (pcd[:, 2] <= z_max)

        combined_mask = x_mask & y_mask & z_mask
        filtered_pcd = pcd[combined_mask]
        return filtered_pcd

    def flatten_processed_pcd(self, pcd_filtered):
        """
        Projects the 3D filtered PCD onto the 2D XY plane by dropping the Z coordinate.
        """
        if pcd_filtered.size == 0:
            return np.empty((0, 2), dtype=np.float32)

        pcd_flattened = pcd_filtered[:, :2]
        return pcd_flattened

    def update_global_grid(self, pcd_flattened_odom):
        """
        Incrementally updates the GLOBAL occupancy grid using transformed points.
        The input pcd_flattened_odom is already in the global map frame (odom).
        """
        if pcd_flattened_odom.shape[0] == 0:
            return

        # 1. Map points to global grid cell indices (i, j)
        pcd_relative = pcd_flattened_odom - np.array([self.map_x_min, self.map_y_min])
        cell_indices_f = pcd_relative / self.resolution
        cell_indices = np.floor(cell_indices_f).astype(int)

        # 2. Clip indices to ensure they fall within the fixed global grid bounds
        rows_idx = np.clip(cell_indices[:, 1], 0, self.rows - 1)
        cols_idx = np.clip(cell_indices[:, 0], 0, self.cols - 1)

        # 3. Update the global log-odds grid for occupied cells
        np.add.at(self.global_occupancy_grid_log, (rows_idx, cols_idx), self.log_odds_hit)

        # 4. Optional: Clamp log-odds values to prevent overflow and extreme certainty
        # self.global_occupancy_grid_log = np.clip(self.global_occupancy_grid_log, -5.0, 5.0)

    def process_scan(self, raw_pcd, x_min, x_max, y_min, y_max, z_min, z_max):
        """
        Runs the full processing pipeline: Filter -> Flatten -> Transform -> Update Grid.
        """
        # 1. Filter points based on the local sensor frame bounds
        pcd_filtered = self.filter_pcd_by_xyz(raw_pcd, x_min, x_max, y_min, y_max, z_min, z_max)

        # 2. Project the 3D points onto the XY plane (still in sensor frame)
        pcd_flattened_sensor = self.flatten_processed_pcd(pcd_filtered)

        # 3. Check for pose and Transform points to Global Frame
        if self.last_robot_pose is None:
            rospy.logwarn("Robot pose is unavailable. Cannot transform points to global frame. Skipping scan update.")
            return

        # Create homogeneous coordinates: (N, 4) array where last column is 1.0
        # Z is assumed to be 0 for the 2D map in the base_link frame
        points_homo = np.ones((pcd_flattened_sensor.shape[0], 4), dtype=np.float32)
        points_homo[:, :2] = pcd_flattened_sensor  # X, Y
        points_homo[:, 2] = 0.0  # Z is 0 in the 2D plane

        # Get the transformation matrix T_odom_base
        T_odom_base = pose_to_transform_matrix(self.last_robot_pose)

        # Apply transformation: P_odom = T_odom_base @ P_base
        # (T_odom_base is 4x4, points_homo is Nx4, need to transpose points for multiplication)
        points_odom_homo = (T_odom_base @ points_homo.T).T

        # Extract the transformed 2D points (X, Y)
        pcd_flattened_odom = points_odom_homo[:, :2]

        # 4. Update the global occupancy grid
        self.update_global_grid(pcd_flattened_odom)

    def save_map(self, filepath=FINAL_SAVE_PATH):
        """
        Saves the current GLOBAL log-odds grid (converted to probability)
        to the specified file path.
        """
        # Convert log-odds to probability before saving
        probability_grid = 1.0 - (1.0 / (1.0 + np.exp(self.global_occupancy_grid_log)))

        if probability_grid.size > 0:
            np.save(filepath, probability_grid)
            rospy.loginfo(f"[SAVE] Successfully saved final global map to: {os.path.abspath(filepath)}")
            rospy.loginfo(f"[SAVE] Map dimensions: {probability_grid.shape}")
        else:
            rospy.logwarn("[SAVE] Warning: Global map is empty. Cannot save.")


# Global publisher instance, initialized in main()
map_publisher = None


def publish_occupancy_grid(processor_instance, timestamp):
    """
    Converts the internal GLOBAL log-odds grid to a nav_msgs/OccupancyGrid message
    and publishes it.
    """
    global map_publisher

    # Check if the publisher is initialized and map data exists
    if map_publisher is None:
        return

    # Convert the GLOBAL log-odds grid to probability and flip for visualization
    probability_grid_global = 1.0 - (1.0 / (1.0 + np.exp(processor_instance.global_occupancy_grid_log)))
    prob_flat = np.flipud(probability_grid_global).flatten()

    # 1. Create the OccupancyGrid message
    grid_msg = OccupancyGrid()

    # 2. Header
    header = Header()
    header.stamp = timestamp
    header.frame_id = GLOBAL_MAP_FRAME  # Map frame name
    grid_msg.header = header

    # 3. Metadata
    grid_msg.info.resolution = processor_instance.resolution

    # Map dimensions (rows=height, cols=width) - NOW FIXED
    grid_msg.info.width = processor_instance.cols
    grid_msg.info.height = processor_instance.rows

    # Map Origin (bottom-left corner of the fixed global map)
    grid_msg.info.origin = Pose(
        position=Point(x=processor_instance.map_x_min, y=processor_instance.map_y_min, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),  # No rotation
    )

    # 4. Data conversion (Probability [0.0, 1.0] -> ROS OccupancyGrid [-1, 0, 100])
    occupancy_data = np.full(prob_flat.shape, -1, dtype=np.int8)

    # Set occupied (prob > 0.65)
    occupancy_data[prob_flat > 0.65] = 100

    # Set free (prob < 0.35)
    occupancy_data[prob_flat < 0.35] = 0

    grid_msg.data = occupancy_data.tolist()

    # 5. Publish
    map_publisher.publish(grid_msg)

    rospy.loginfo(
        f"| PUBLISH | OccupancyGrid message published to {MAP_TOPIC}. Shape: {processor_instance.rows}x{processor_instance.cols}"
    )


# --- ROS Callback and Main Execution ---

# Global processor instance
processor = PCDProcessor(grid_resolution=GRID_RESOLUTION)
scan_counter = 0


def odom_callback(msg):
    """
    ROS Callback function for Odometry messages. Stores the robot's current pose.
    """
    processor.set_robot_pose(msg.pose.pose)
    # The debug log is commented out to reduce console clutter
    # rospy.loginfo(f"[ODOM] Pose Updated: X={msg.pose.pose.position.x:.2f}, Y={msg.pose.pose.position.y:.2f}")


def pcd_callback(msg):
    """
    ROS Callback function, executed whenever a new PCD message is received.
    Processes the scan and updates the global occupancy grid.
    """
    global scan_counter
    scan_counter += 1

    start_time = time.time()

    # 1. Convert the ROS message to the standard (N, 3) NumPy array
    if INPUT_SOURCE_TYPE == "POINTCLOUD2":
        raw_pcd = convert_pointcloud2_to_numpy(msg)
    elif INPUT_SOURCE_TYPE == "LIVOX_CUSTOM_MSG":
        raw_pcd = convert_livox_custom_to_numpy(msg)
    else:
        rospy.logerr(f"Unsupported INPUT_SOURCE_TYPE: {INPUT_SOURCE_TYPE}. Skipping scan.")
        return

    # Handle case where conversion results in an empty array
    if raw_pcd.size == 0:
        rospy.logwarn("Received scan resulted in an empty NumPy array after conversion. Skipping.")
        return

    # 2. Process the scan (filtering, flattening, transformation, and grid update)
    processor.process_scan(
        raw_pcd, X_LOWER_LIMIT, X_UPPER_LIMIT, Y_LOWER_LIMIT, Y_UPPER_LIMIT, Z_LOWER_LIMIT, Z_UPPER_LIMIT
    )

    end_time = time.time()

    # 3. Publish the OccupancyGrid map
    rospy.loginfo(f"--- Scan {scan_counter} Processed in {(end_time - start_time):.4f}s ---")

    # Publish the map using the timestamp from the incoming PCD message header
    publish_occupancy_grid(processor, msg.header.stamp)


def main():
    """
    Initializes the ROS node, sets up the subscriber, and initializes the publisher.
    """
    global map_publisher

    # 1. Initialize ROS node
    rospy.init_node("pcd_occupancy_mapper", anonymous=True)
    rospy.loginfo(f"--- Node Initialized: Occupancy Mapper ({INPUT_SOURCE_TYPE}) ---")

    # 2. Determine PCD topic and message type based on configuration
    if INPUT_SOURCE_TYPE == "POINTCLOUD2":
        topic = "/livox/lidar_fov"
        msg_type = PointCloud2
    elif INPUT_SOURCE_TYPE == "LIVOX_CUSTOM_MSG":
        topic = "/livox/lidar_fov"
        msg_type = CustomMsg
    else:
        rospy.logerr(f"Invalid INPUT_SOURCE_TYPE configured: {INPUT_SOURCE_TYPE}. Exiting.")
        return

    # 3. Initialize Publisher
    map_publisher = rospy.Publisher(MAP_TOPIC, OccupancyGrid, queue_size=1)
    rospy.loginfo(f"Initialized map publisher on topic: {MAP_TOPIC}")

    # 4. Create PCD subscriber
    rospy.Subscriber(topic, msg_type, pcd_callback)
    rospy.loginfo(f"Subscribed to PCD topic: {topic} using message type {msg_type.__name__}")

    # 5. Create Odom subscriber (NEW)
    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback)
    rospy.loginfo(f"Subscribed to Odom topic: {ODOM_TOPIC}")

    # 6. Register shutdown hook to save the map
    rospy.on_shutdown(processor.save_map)

    # 7. Start processing callbacks
    try:
        rospy.spin()  # Blocks until node is shut down
    except rospy.ROSInterruptException:
        pass  # Allow cleanup to proceed

    # --- Termination Action is now handled by rospy.on_shutdown ---
    rospy.loginfo("--- Node Shutdown Complete ---")


if __name__ == "__main__":
    # In a live ROS setup, if this is run directly, it will execute the main function.
    main()
