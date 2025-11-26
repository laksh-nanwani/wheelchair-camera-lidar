import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import time
from collections import defaultdict
import sys  # <-- Added import
import os
import yaml
from matplotlib.pyplot import imsave
from scipy import ndimage


def filter_by_voxel_density(pcd, voxel_size, min_points_per_voxel):
    """
    Removes sparse points (like 'ghost' trails) from a point cloud.

    It divides the space into voxels, counts points in each, and keeps
    only the points that belong to voxels with at least
    `min_points_per_voxel`.

    Args:
        pcd (o3d.geometry.PointCloud): The input point cloud.
        voxel_size (float): The side length (in meters) of each 3D voxel.
        min_points_per_voxel (int): The minimum number of points a voxel
                                    must contain for its points to be kept.

    Returns:
        o3d.geometry.PointCloud: The filtered point cloud.
    """
    print("Starting Voxel Density Filter...")
    print(f"  Voxel Size: {voxel_size}m")
    print(f"  Min Points per Voxel: {min_points_per_voxel}")

    start_time = time.time()

    # 1. Get points and find voxel index for each point
    points = np.asarray(pcd.points)
    points_min = points.min(axis=0)

    # (N, 3) array of integer voxel coordinates
    voxel_indices = np.floor((points - points_min) / voxel_size).astype(int)

    # 2. Group point indices by their voxel coordinate
    # Use a dictionary { (vx, vy, vz) -> [list of point indices] }
    voxel_map = defaultdict(list)
    for i in range(len(voxel_indices)):
        voxel_map[tuple(voxel_indices[i])].append(i)

    # 3. Collect all point indices that are in "dense" voxels
    keep_indices = []

    for point_indices_in_voxel in voxel_map.values():
        if len(point_indices_in_voxel) >= min_points_per_voxel:
            keep_indices.extend(point_indices_in_voxel)

    # 4. Create a new point cloud with only the "dense" points
    filtered_pcd = pcd.select_by_index(keep_indices)

    end_time = time.time()
    num_removed = len(points) - len(keep_indices)
    print(f"  ...Voxel filter done in {end_time - start_time:.2f}s.")
    print(f"  Removed {num_removed} sparse points ({num_removed / len(points) * 100:.1f}%).")

    return filtered_pcd


def filter_by_statistical_outlier(pcd, nb_neighbors, std_ratio):
    """
    Removes sparse outlier points (like 'ghost' trails) using
    Statistical Outlier Removal (SOR).

    For each point, it computes the average distance to its 'nb_neighbors'
    nearest neighbors. It then removes points that are further than
    'std_ratio' standard deviations away from the mean of all such distances.

    Args:
        pcd (o3d.geometry.PointCloud): The input point cloud.
        nb_neighbors (int): Number of neighbors to analyze for each point.
        std_ratio (float): The standard deviation ratio. Lower values
                           make the filter more aggressive (remove more points).

    Returns:
        o3d.geometry.PointCloud: The point cloud with outliers removed.
    """
    print(f"\nStarting Statistical Outlier Removal...")
    print(f"  nb_neighbors: {nb_neighbors}")
    print(f"  std_ratio: {std_ratio}")
    start_time = time.time()

    # The function returns the downsampled_pcd and the indices
    # We just need the pcd
    filtered_pcd, inlier_indices = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    end_time = time.time()
    num_original = len(np.asarray(pcd.points))
    num_filtered = len(np.asarray(filtered_pcd.points))
    num_removed = num_original - num_filtered

    print(f"  ...SOR done in {end_time - start_time:.2f}s.")
    print(f"  Removed {num_removed} outlier points ({num_removed / num_original * 100:.1f}%).")

    return filtered_pcd


def patchwork_to_occupancy(
    pcd,
    patch_size=1.0,
    grid_resolution=0.1,
    ground_threshold=0.15,
    max_obstacle_height=2.0,
    ransac_dist_threshold=0.1,
    ransac_min_points=30,
    max_ground_angle_deg=30.0,
):
    """
    Converts a 3D point cloud to a 2D occupancy grid using the Patchwork method.

    This method is robust to uneven ground, slopes, and sensor tilt.

    Args:
        pcd (o3d.geometry.PointCloud): The input Open3D point cloud.
        patch_size (float): The side length (in meters) of the square patches.
        grid_resolution (float): The side length (in meters) of each cell in the final occupancy grid.
        ground_threshold (float): Max distance (in meters) from a point to its local ground plane
                                  to be considered "ground".
        max_obstacle_height (float): Max height (in meters) above the local ground for a point
                                     to be considered an obstacle. Points above this are ignored
                                     (e.g., ceilings, high branches).
        ransac_dist_threshold (float): RANSAC distance threshold for plane fitting.
        ransac_min_points (int): Minimum number of points required in a patch to attempt RANSAC.
        max_ground_angle_deg (float): The maximum angle (in degrees) the ground plane's normal
                                      can deviate from the Z-axis. This prevents fitting to walls.

    Returns:
        tuple: (occupancy_grid, obstacle_pcd, ground_pcd, grid_origin_xy, grid_shape)
            - occupancy_grid (np.array): The 2D (rows, cols) occupancy grid (1=occupied, 0=free).
            - obstacle_pcd (o3d.geometry.PointCloud): Point cloud of classified obstacles.
            - ground_pcd (o3d.geometry.PointCloud): Point cloud of classified ground.
            - grid_origin_xy (tuple): (x_min, y_min) coordinates of the grid's (0,0) corner.
            - grid_shape (tuple): (rows, cols) of the grid.
    """

    print("Starting Patchwork Occupancy Grid Generation...")
    start_time = time.time()

    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        print("Error: Empty point cloud.")
        return None, None, None, None, None

    # --- 1. Define Grid Boundaries (ROS/Image convention) ---
    x_min, y_min, _ = np.min(points, axis=0)
    x_max, y_max, _ = np.max(points, axis=0)

    # Use your grid sizing logic
    grid_cols = int((x_max - x_min) / grid_resolution) + 1
    grid_rows = int((y_max - y_min) / grid_resolution) + 1
    grid_shape = (grid_rows, grid_cols)

    # The origin (x_min, y_min) now corresponds to grid cell (0, grid_rows - 1)
    grid_origin_xy = (x_min, y_min)

    if grid_rows == 0 or grid_cols == 0:
        print("Error: Invalid grid dimensions.")
        return None, None, None, None, None

    # Default to 0 (free). ROS maps often use 0-100 or -1 (unknown),
    # but for a binary grid, 0=free, 1=occupied is clear.
    # occupancy_grid = np.zeros(grid_shape, dtype=np.uint8)
    occupancy_grid = np.full(grid_shape, 0.5, dtype=np.float32)

    # --- 2. Pass 1: Bin points into patches ---
    print(f"Pass 1: Binning {points.shape[0]} points into {patch_size}m patches...")
    patches = defaultdict(list)
    for i in range(points.shape[0]):
        patch_x = int(np.floor(points[i, 0] / patch_size))
        patch_y = int(np.floor(points[i, 1] / patch_size))
        patches[(patch_x, patch_y)].append(i)  # Store point index

    # --- 3. Pass 2: Build Local Ground Model for each patch ---
    print(f"Pass 2: Building ground model for {len(patches)} patches...")
    ground_model = {}  # Stores (model_type, data)

    # Z-axis vector for angle constraint
    z_axis = np.array([0.0, 0.0, 1.0])
    max_angle_rad = np.deg2rad(max_ground_angle_deg)

    for (px, py), indices in patches.items():
        patch_points = points[indices]

        if patch_points.shape[0] < ransac_min_points:
            # Not enough points, use robust median Z as fallback
            if patch_points.shape[0] > 0:
                z_fallback = np.median(patch_points[:, 2])
                ground_model[(px, py)] = ("height", z_fallback)
            continue

        # We have enough points, try RANSAC plane fitting
        patch_pcd = o3d.geometry.PointCloud()
        patch_pcd.points = o3d.utility.Vector3dVector(patch_points)

        try:
            # Fit plane with a Z-axis constraint to avoid fitting to walls
            plane_model, inliers = patch_pcd.segment_plane(
                distance_threshold=ransac_dist_threshold, ransac_n=3, num_iterations=100
            )

            # Check plane normal angle
            normal = plane_model[:3]
            normal = normal / np.linalg.norm(normal)
            # angle_rad = np.arccos(np.dot(normal, z_axis))
            # Get dot product, take abs() to handle normals pointing up or down
            dot_product = abs(np.dot(normal, z_axis))
            # acos will now always be between 0 (flat) and pi/2 (vertical)
            angle_rad = np.arccos(dot_product)

            if abs(angle_rad) < max_angle_rad:
                # Good plane, store it
                ground_model[(px, py)] = ("plane", plane_model)
            else:
                # Plane is too vertical (likely a wall), use fallback
                z_fallback = np.median(patch_points[inliers, 2])
                ground_model[(px, py)] = ("height", z_fallback)

        except Exception as e:
            # RANSAC failed, use robust median Z of all patch points
            # print(f"RANSAC failed for patch ({px}, {py}): {e}")
            if patch_points.shape[0] > 0:
                z_fallback = np.median(patch_points[:, 2])
                ground_model[(px, py)] = ("height", z_fallback)

    # --- 4. Pass 3: Classify all points and build occupancy grid ---
    print("Pass 3: Classifying points and building grid...")
    obstacle_indices = []
    ground_indices = []

    for i in range(points.shape[0]):
        point = points[i]
        x, y, z = point

        patch_x = int(np.floor(x / patch_size))
        patch_y = int(np.floor(y / patch_size))

        if (patch_x, patch_y) in ground_model:
            model_type, model_data = ground_model[(patch_x, patch_y)]

            z_ground = 0.0
            if model_type == "plane":
                a, b, c, d = model_data
                if c == 0:
                    c = 1e-6  # Avoid division by zero
                z_ground = -(a * x + b * y + d) / c
            else:  # model_type == "height"
                z_ground = model_data

            z_relative = z - z_ground

            # --- Classification ---
            if ground_threshold <= z_relative < max_obstacle_height:
                # This is an OBSTACLE point
                obstacle_indices.append(i)

                # Project to 2D occupancy grid (using your ROS/image convention)
                grid_c = int((x - x_min) / grid_resolution)
                # Flip Y-axis: (0,0) is top-left
                grid_r = (grid_rows - 1) - int((y - y_min) / grid_resolution)

                if 0 <= grid_r < grid_rows and 0 <= grid_c < grid_cols:
                    occupancy_grid[grid_r, grid_c] = 1  # Mark as occupied

            # elif abs(z_relative) < ground_threshold:
            #     # This is a GROUND point
            #     ground_indices.append(i)
            elif abs(z_relative) < ground_threshold:
                # This is a GROUND point
                ground_indices.append(i)

                # Project to 2D occupancy grid (using your ROS/image convention)
                grid_c = int((x - x_min) / grid_resolution)
                grid_r = (grid_rows - 1) - int((y - y_min) / grid_resolution)

                if 0 <= grid_r < grid_rows and 0 <= grid_c < grid_cols:
                    # Only mark as 'free' (0.0) if not already 'occupied' (1.0)
                    if occupancy_grid[grid_r, grid_c] != 1.0:
                        occupancy_grid[grid_r, grid_c] = 0.0
            # else:
            # Point is too high (ignored) or too low (noise, ignored)

        # else:
        # Point is in a patch that had no model (e.g., < min_points)
        # We ignore this point as we have no ground reference

    # --- 5. Create final outputs ---
    print("Finalizing outputs...")
    obstacle_points = points[obstacle_indices]
    ground_points = points[ground_indices]

    obstacle_pcd = o3d.geometry.PointCloud()
    obstacle_pcd.points = o3d.utility.Vector3dVector(obstacle_points)
    obstacle_pcd.paint_uniform_color([1.0, 0, 0])  # Red

    ground_pcd = o3d.geometry.PointCloud()
    ground_pcd.points = o3d.utility.Vector3dVector(ground_points)
    ground_pcd.paint_uniform_color([0, 0.6, 0.2])  # Green

    end_time = time.time()
    print(f"Done. Total time: {end_time - start_time:.2f}s")

    return occupancy_grid, obstacle_pcd, ground_pcd, grid_origin_xy, grid_shape


def save_occupancy(grid, res, out_path, map_name, origin):
    """
    Saves the occupancy grid as a PNG image.

    Args:
        grid (np.array): 2D occupancy grid (1=occupied, 0=free).
        res (float): Resolution of each cell in meters.
        path (str): File path to save the PNG image.
    """
    grid_ = ((1 - grid) * 255).astype(int)
    x_min, y_min = origin

    map_dict = {
        "image": f"{map_name}.png",
        "resolution": res,
        "origin": [float(x_min), float(y_min), 0.0],
        "occupied_thresh": 0.6,
        "free_thresh": 0.3,
        "negate": 0,
    }

    imsave(os.path.join(out_path, f"{map_name}.png"), grid_, cmap="gray")

    with open(os.path.join(out_path, f"{map_name}.yaml"), "w") as file:
        yaml.dump(map_dict, file, default_flow_style=None)

    print("Saved map to", out_path)


def smooth_free_space(grid, kernel_size=3):
    """
    Cleans up 'patchy' free space by filling holes (e.g., from sparse
    LiDAR ground returns) using a morphological closing operation.

    This function is designed to "bridge" gaps between 'free' (0.0) areas,
    turning 'unknown' (0.5) gaps into 'free' (0.0).

    Args:
        grid (np.array): The 2D occupancy grid (0.0, 0.5, 1.0).
        kernel_size (int): The size of the filter (e.g., 3 for 3x3).
                           This should be scaled to the size of the
                           gaps you want to fill.

    Returns:
        np.array: The smoothed 2D occupancy grid.
    """
    print(f"\nSmoothing free space by 'closing' gaps with {kernel_size}x{kernel_size} kernel...")

    # 1. Find all obstacles. We will preserve these unconditionally.
    obstacle_mask = grid == 1.0

    # 2. Find all 'free' (0.0) areas. These are what we want to "grow"
    #    to fill the 'unknown' gaps.
    free_mask = grid == 0.0

    # 3. Create the filter kernel
    structure = np.ones((kernel_size, kernel_size))

    # 4. Apply 'binary_closing' to the 'free' mask.
    # This 'dilates' the free space (filling gaps) and then
    # 'erodes' it back, effectively "bridging" nearby free regions.
    filled_free_mask = ndimage.binary_closing(free_mask, structure=structure)

    # 5. Reconstruct the grid
    # Start with 0.5 (unknown) everywhere
    smoothed_grid = np.full(grid.shape, 0.5, dtype=np.float32)

    # Add back the new, 'filled' free mask
    smoothed_grid[filled_free_mask] = 0.0

    # Add back all obstacles (which always have priority)
    smoothed_grid[obstacle_mask] = 1.0

    print("...Smoothing done.")
    return smoothed_grid


# --- --- --- --- --- --- --- --- --- --- --- ---
# --- DEMO: Load a real PCD file and test    ---
# --- --- --- --- --- --- --- --- --- --- --- ---

# Removed the create_demo_world() function

if __name__ == "__main__":
    # --- 1. Load Point Cloud from file ---
    # if len(sys.argv) < 2:
    #     print("Error: No .pcd file provided.")
    #     print("Usage: python patchwork_occupancy.py <path_to_pcd_file>")
    #     sys.exit(1)
    pcd_name = f"himalaya+kcis"
    pcd_path = f"/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/PCD/{pcd_name}.pcd"

    grid_path = "/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/maps_patchwork/"

    # pcd_path = sys.argv[1]
    print(f"Loading point cloud from {pcd_path}...")

    try:
        demo_pcd = o3d.io.read_point_cloud(pcd_path)
        if not demo_pcd.has_points():
            print(f"Error: Point cloud file is empty or could not be read: {pcd_path}")
            sys.exit(1)
    except Exception as e:
        print(f"Error loading {pcd_path}: {e}")
        sys.exit(1)

    print(f"Successfully loaded {len(demo_pcd.points)} points.")

    # --- Density Filter Hyperparameters (Tune these!) ---
    # Good starting point: voxel size slightly larger than your grid resolution.
    DENSITY_VOXEL_SIZE = 0.1  # 10cm cube
    # This is the key parameter. Higher = more aggressive filtering.
    # Start with a low number and increase it.
    MIN_POINTS_PER_VOXEL = 10

    filtered_pcd = filter_by_voxel_density(
        demo_pcd, voxel_size=DENSITY_VOXEL_SIZE, min_points_per_voxel=MIN_POINTS_PER_VOXEL
    )

    # # --- SOR Hyperparameters (Tune these!) ---

    # # nb_neighbors: How many neighbors to check for density.
    # # (e.g., 20-50)
    # SOR_NEIGHBORS = 30

    # # std_ratio: The aggressiveness. Lower is more aggressive.
    # # 2.0 is a good default. 1.0 is very aggressive. 3.0 is mild.
    # SOR_STD_RATIO = 2.0

    # filtered_pcd = filter_by_statistical_outlier(demo_pcd, nb_neighbors=SOR_NEIGHBORS, std_ratio=SOR_STD_RATIO)

    # Optional: Visualize what the filter did
    # print("\nVisualizing original vs. filtered (Press 'Q' to close)")
    o3d.visualization.draw_geometries([demo_pcd], window_name="Original")
    o3d.visualization.draw_geometries([filtered_pcd], window_name="Filtered")

    # 2. Run the Patchwork algorithm
    # Hyperparameters (you can tune these)
    PATCH_SIZE = 0.5  # 2x2m patches
    GRID_RESOLUTION = 0.05  # 10cm grid cells
    GROUND_THRESHOLD = 0.05  # 10cm tolerance for ground
    MAX_OBSTACLE_HEIGHT = 1.5  # Ignore points > 2m above ground
    RANSAC_DIST_THRESHOLD = 0.05  # 10cm RANSAC tolerance
    RANSAC_MIN_POINTS = 30  # Min points in a patch for RANSAC
    MAX_GROUND_ANGLE_DEG = 6.0  # Max angle for a plane to be "ground"

    grid, obs_pcd, gnd_pcd, origin, shape = patchwork_to_occupancy(
        filtered_pcd,
        patch_size=PATCH_SIZE,
        grid_resolution=GRID_RESOLUTION,
        ground_threshold=GROUND_THRESHOLD,
        max_obstacle_height=MAX_OBSTACLE_HEIGHT,
        ransac_dist_threshold=RANSAC_DIST_THRESHOLD,
        ransac_min_points=RANSAC_MIN_POINTS,
        max_ground_angle_deg=MAX_GROUND_ANGLE_DEG,
    )

    # 3. Visualize the 3D classified result
    if obs_pcd and gnd_pcd:
        print("\nVisualizing classified 3D points (Green=Ground, Red=Obstacle)...")
        # Add a coordinate frame
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        o3d.visualization.draw_geometries([gnd_pcd, obs_pcd, coord_frame], window_name="Classified Points (Patchwork)")

    # 4. Visualize the 2D Occupancy Grid
    if grid is not None:
        # Automatically calculate kernel size to fill gaps.
        # We want a kernel slightly larger than the patch-induced gap.
        # Gap size in pixels = PATCH_SIZE / GRID_RESOLUTION
        # Add +3 for a good buffer and to ensure it's an odd number.
        gap_size_pixels = int(PATCH_SIZE / GRID_RESOLUTION)
        SMOOTHING_KERNEL_SIZE = gap_size_pixels + 3

        # Ensure it's an odd number
        if SMOOTHING_KERNEL_SIZE % 2 == 0:
            SMOOTHING_KERNEL_SIZE += 1

        grid = smooth_free_space(grid, kernel_size=SMOOTHING_KERNEL_SIZE)

        save_occupancy(grid, GRID_RESOLUTION, grid_path, pcd_name, origin)

        print("\nVisualizing final 2D Occupancy Grid...")
        plt.figure(figsize=(10, 10))
        # We use 'origin' to set the correct axes labels
        extent = [origin[0], origin[0] + shape[1] * GRID_RESOLUTION, origin[1], origin[1] + shape[0] * GRID_RESOLUTION]

        # We NO LONGER flip the grid, as the ROS convention logic already did it.
        # plt.imshow(grid, cmap="gray_r", extent=extent)
        plt.imshow(grid, cmap="gray_r", extent=extent, vmin=0.0, vmax=1.0)

        plt.title(f"2D Occupancy Grid (Resolution: {GRID_RESOLUTION}m)")
        plt.xlabel("X Coordinate (meters)")
        plt.ylabel("Y Coordinate (meters)")
        plt.gca().set_aspect("equal", adjustable="box")
        plt.show()
