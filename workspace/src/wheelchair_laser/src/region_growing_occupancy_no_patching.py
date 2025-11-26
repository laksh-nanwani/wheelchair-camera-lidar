import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import time
from collections import defaultdict, deque
import sys
import os
import yaml
from matplotlib.pyplot import imsave
from scipy import ndimage


# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
# --- STEP 1: PRE-FILTERING / DOWNSAMPLING ---
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---

# NOTE: I have removed 'filter_by_voxel_density' as it was not the
# right tool for this problem. 'voxel_down_sample' (used in __main__)
# is the correct way to handle a dense cloud and fix the slowness.


def filter_by_statistical_outlier(pcd, nb_neighbors, std_ratio):
    """
    Removes sparse outlier points (like 'ghost' trails) using
    Statistical Outlier Removal (SOR).
    """
    print(f"\nStarting Statistical Outlier Removal...")
    print(f"  nb_neighbors: {nb_neighbors}")
    print(f"  std_ratio: {std_ratio}")
    start_time = time.time()

    filtered_pcd, inlier_indices = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    end_time = time.time()
    num_original = len(np.asarray(pcd.points))
    num_filtered = len(np.asarray(filtered_pcd.points))
    num_removed = num_original - num_filtered

    print(f"  ...SOR done in {end_time - start_time:.2f}s.")
    print(f"  Removed {num_removed} outlier points ({num_removed / num_original * 100:.1f}%).")

    return filtered_pcd


# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
# --- STEP 2: 3D REGION-GROWING (NEW, NO PATCHING) ---
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---


def region_growing_pcd(
    pcd,
    normal_estimation_radius=0.1,
    normal_estimation_nn=30,
    neighbor_search_radius=0.2,
    normal_angle_threshold_deg=10.0,
    seed_selection_method="origin",  # 'lowest' or 'origin'
    origin_search_radius=1.0,
    seed_max_ground_angle_deg=45.0,  # For smart seed finding
):
    """
    Segments the point cloud into ground and obstacles using direct
    3D region growing based on normal similarity.
    """

    print("\nStarting 3D Region Growing (No Patching)...")
    start_time = time.time()

    # 1. Pre-processing: Calculate Normals
    # This is now MUCH faster on the downsampled cloud
    if not pcd.has_normals():
        print(f"  Calculating normals (radius={normal_estimation_radius}, max_nn={normal_estimation_nn})...")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=normal_estimation_radius, max_nn=normal_estimation_nn
            )
        )

    pcd.orient_normals_consistent_tangent_plane(k=normal_estimation_nn)

    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)

    if points.shape[0] == 0:
        print("Error: Empty cloud.")
        return None, None

    # 2. Build KD-Tree for neighbor search
    print("  Building KD-Tree for neighbor search...")
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)

    # 3. Find Seed Point
    seed_idx = -1
    if seed_selection_method == "lowest":
        seed_idx = np.argmin(points[:, 2])
        print(f"  Using 'lowest' seed point (Index: {seed_idx}, Z: {points[seed_idx, 2]:.2f})")

    else:  # 'origin' - NEW SmaRT SEED LOGIC
        # Find points near (0,0)
        origin_point = np.array([0.0, 0.0, np.min(points[:, 2])])
        [k, seed_idx_list, _] = pcd_tree.search_radius_vector_3d(origin_point, origin_search_radius)

        if k > 0:
            # We found points near the origin.
            # Now, find the one that is the *best* ground candidate.
            neighbor_normals = normals[seed_idx_list]

            # Calculate dot product with Z-axis [0,0,1]
            z_dot_product = np.abs(neighbor_normals[:, 2])

            # Find points that are "ground-like" (not walls)
            ground_like_threshold = np.cos(np.deg2rad(seed_max_ground_angle_deg))
            ground_indices = np.where(z_dot_product > ground_like_threshold)[0]

            if len(ground_indices) > 0:
                # We have at least one ground-like point
                # Find the one *most* like ground (normal closest to [0,0,1])
                best_relative_idx = np.argmax(z_dot_product[ground_indices])
                best_idx_in_list = ground_indices[best_relative_idx]
                seed_idx = seed_idx_list[best_idx_in_list]
                print(
                    f"  Using 'smart' origin seed (Index: {seed_idx}, Z-Normal: {z_dot_product[best_idx_in_list]:.3f})"
                )
            else:
                # No ground-like points found, fall back to lowest in radius
                print("  Warning: No 'ground-like' points near origin. Falling back to lowest point in radius.")
                origin_points_z = points[seed_idx_list, 2]
                seed_idx = seed_idx_list[np.argmin(origin_points_z)]

        if seed_idx == -1:
            # k == 0 or other fallbacks failed. Use global lowest.
            print("  Warning: No points found near origin. Falling back to 'lowest' point.")
            seed_idx = np.argmin(points[:, 2])

    if seed_idx == -1:
        print("Error: Could not find a valid seed point.")
        return None, None

    # 4. Perform Region Growing (BFS)
    print(f"  Starting region growing (BFS) from seed {seed_idx}...")
    print(f"  Normal Angle Threshold: {normal_angle_threshold_deg} deg")
    print(f"  Neighbor Search Radius: {neighbor_search_radius} m")

    queue = deque([seed_idx])
    ground_indices = set()
    visited = np.zeros(len(points), dtype=bool)

    visited[seed_idx] = True
    max_angle_rad = np.deg2rad(normal_angle_threshold_deg)

    while queue:
        current_idx = queue.popleft()
        ground_indices.add(current_idx)
        current_normal = normals[current_idx]

        [k, neighbor_indices, _] = pcd_tree.search_radius_vector_3d(points[current_idx], neighbor_search_radius)

        for n_idx in neighbor_indices:
            if n_idx == current_idx or visited[n_idx]:
                continue

            neighbor_normal = normals[n_idx]
            dot_product = abs(np.dot(current_normal, neighbor_normal))
            dot_product = min(1.0, dot_product)  # Clamp
            angle_rad = np.arccos(dot_product)

            if angle_rad < max_angle_rad:
                visited[n_idx] = True
                queue.append(n_idx)

    print(f"  ...Region growing found {len(ground_indices)} ground points.")

    # 5. Final Classification
    all_indices = set(range(len(points)))
    obstacle_indices = list(all_indices - ground_indices)
    ground_indices_list = list(ground_indices)

    obstacle_pcd = pcd.select_by_index(obstacle_indices)
    obstacle_pcd.paint_uniform_color([1.0, 0, 0])  # Red

    ground_pcd = pcd.select_by_index(ground_indices_list)
    ground_pcd.paint_uniform_color([0, 0.6, 0.2])  # Green

    end_time = time.time()
    print(f"  ...3D Segmentation done in {end_time - start_time:.2f}s")

    return ground_pcd, obstacle_pcd


# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
# --- STEP 3: 2D GRID PROJECTION & POST-PROCESSING (From your original code) ---
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---


def create_occupancy_grid_from_pcds(gnd_pcd, obs_pcd, grid_resolution):
    """
    Creates a 2D occupancy grid from classified ground and obstacle
    point clouds.
    """
    print(f"\nStarting 2D Grid Projection (Resolution: {grid_resolution}m)...")

    ground_points = np.asarray(gnd_pcd.points)
    obstacle_points = np.asarray(obs_pcd.points)

    if ground_points.shape[0] == 0 and obstacle_points.shape[0] == 0:
        print("  Error: No ground or obstacle points to project.")
        return None, None, None

    if ground_points.shape[0] == 0:
        all_points = obstacle_points
    elif obstacle_points.shape[0] == 0:
        all_points = ground_points
    else:
        all_points = np.vstack((ground_points, obstacle_points))

    # --- 1. Define Grid Boundaries (ROS/Image convention) ---
    x_min, y_min, _ = np.min(all_points, axis=0)
    x_max, y_max, _ = np.max(all_points, axis=0)

    grid_cols = int((x_max - x_min) / grid_resolution) + 1
    grid_rows = int((y_max - y_min) / grid_resolution) + 1
    grid_shape = (grid_rows, grid_cols)
    grid_origin_xy = (x_min, y_min)

    if grid_rows == 0 or grid_cols == 0:
        print("  Error: Invalid grid dimensions.")
        return None, None, None

    print(f"  Grid Dimensions: {grid_rows} rows x {grid_cols} cols")
    occupancy_grid = np.full(grid_shape, 0.5, dtype=np.float32)

    # --- 2. Project Ground Points (Free Space) ---
    if ground_points.shape[0] > 0:
        grid_c_gnd = ((ground_points[:, 0] - x_min) / grid_resolution).astype(int)
        grid_r_gnd = (grid_rows - 1) - ((ground_points[:, 1] - y_min) / grid_resolution).astype(int)
        grid_c_gnd = np.clip(grid_c_gnd, 0, grid_cols - 1)
        grid_r_gnd = np.clip(grid_r_gnd, 0, grid_rows - 1)
        occupancy_grid[grid_r_gnd, grid_c_gnd] = 0.0

    # --- 3. Project Obstacle Points (Occupied Space) ---
    if obstacle_points.shape[0] > 0:
        grid_c_obs = ((obstacle_points[:, 0] - x_min) / grid_resolution).astype(int)
        grid_r_obs = (grid_rows - 1) - ((obstacle_points[:, 1] - y_min) / grid_resolution).astype(int)
        grid_c_obs = np.clip(grid_c_obs, 0, grid_cols - 1)
        grid_r_obs = np.clip(grid_r_obs, 0, grid_rows - 1)
        occupancy_grid[grid_r_obs, grid_c_obs] = 1.0

    print("  ...Grid projection done.")
    return occupancy_grid, grid_origin_xy, grid_shape


def smooth_free_space(grid, kernel_size=3):
    """
    Cleans up 'patchy' free space by filling holes using a
    morphological closing operation. (From your original code)
    """
    print(f"\nSmoothing free space by 'closing' gaps with {kernel_size}x{kernel_size} kernel...")
    obstacle_mask = grid == 1.0
    free_mask = grid == 0.0
    structure = np.ones((kernel_size, kernel_size))
    filled_free_mask = ndimage.binary_closing(free_mask, structure=structure)
    smoothed_grid = np.full(grid.shape, 0.5, dtype=np.float32)
    smoothed_grid[filled_free_mask] = 0.0
    smoothed_grid[obstacle_mask] = 1.0
    print("...Smoothing done.")
    return smoothed_grid


def save_occupancy(grid, res, out_path, map_name, origin):
    """
    Saves the occupancy grid as a PNG image and YAML file (ROS format).
    (From your original code)
    """
    grid_ = ((1 - grid) * 255).astype(np.uint8)
    x_min, y_min = origin

    map_dict = {
        "image": f"{map_name}.png",
        "resolution": res,
        "origin": [float(x_min), float(y_min), 0.0],
        "occupied_thresh": 0.6,
        "free_thresh": 0.3,
        "negate": 0,
    }

    if not os.path.exists(out_path):
        os.makedirs(out_path)
        print(f"Created directory: {out_path}")

    png_path = os.path.join(out_path, f"{map_name}.png")
    imsave(png_path, grid_, cmap="gray")

    yaml_path = os.path.join(out_path, f"{map_name}.yaml")
    with open(yaml_path, "w") as file:
        yaml.dump(map_dict, file, default_flow_style=None)

    print(f"Saved map to {out_path} (PNG and YAML)")


def filter_by_voxel_density(pcd, voxel_size, min_points_per_voxel):
    """
    Removes sparse points (like 'ghost' trails) from a point cloud.
    (FASTER, VECTORIZED VERSION)
    """
    print("Starting Voxel Density Filter ...")
    print(f"  Voxel Size: {voxel_size}m")
    print(f"  Min Points per Voxel: {min_points_per_voxel}")

    start_time = time.time()

    # 1. Get points and find voxel index for each point
    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        print("  ...Voxel filter done. Empty cloud.")
        return pcd

    points_min = points.min(axis=0)

    # (N, 3) array of integer voxel coordinates
    voxel_indices = np.floor((points - points_min) / voxel_size).astype(int)

    # 2. Use numpy to find unique voxels and count points in each
    # This is *much* faster than a Python loop and dictionary
    unique_voxels, point_indices_in_voxel, counts = np.unique(
        voxel_indices, axis=0, return_inverse=True, return_counts=True
    )

    # 3. Find which unique voxels are "dense"
    dense_voxel_indices = np.where(counts >= min_points_per_voxel)[0]

    # 4. Find which *points* belong to these dense voxels
    # `point_indices_in_voxel` maps each point to its `unique_voxels` index
    # We create a mask for points whose index is in `dense_voxel_indices`
    keep_mask = np.isin(point_indices_in_voxel, dense_voxel_indices)

    # 5. Get the indices of the points to keep
    keep_indices = np.where(keep_mask)[0]

    # 6. Create a new point cloud with only the "dense" points
    filtered_pcd = pcd.select_by_index(keep_indices)

    end_time = time.time()
    num_removed = len(points) - len(keep_indices)
    print(f"  ...Voxel filter done in {end_time - start_time:.2f}s.")
    print(f"  Removed {num_removed} sparse points ({num_removed / len(points) * 100:.1f}%).")

    return filtered_pcd


# --- --- --- --- --- --- --- --- --- --- --- ---
# --- MAIN EXECUTION (PIPELINE) ---
# --- --- --- --- --- --- --- --- --- --- --- ---

if __name__ == "__main__":
    # --- 1. Load Point Cloud from file ---
    pcd_name = "nexus_first_floor_with_stores"
    pcd_path = f"/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/PCD/{pcd_name}.pcd"
    grid_path = "/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/maps_3d_region/"

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

    # --- 3. Filter (Optional, on the *downsampled* cloud) ---
    # The 'filter_by_voxel_density' is removed as 'voxel_down_sample'
    # is the correct tool for a dense cloud. You could still run
    # SOR here if needed.
    # filtered_pcd = filter_by_statistical_outlier(downsampled_pcd, ...)
    DENSITY_VOXEL_SIZE = 0.1
    MIN_POINTS_PER_VOXEL = 10
    GRID_RESOLUTION = 0.05  # 5cm grid cells

    filtered_pcd = filter_by_voxel_density(
        demo_pcd,  # <-- IMPORTANT: Use the downsampled cloud
        voxel_size=DENSITY_VOXEL_SIZE,
        min_points_per_voxel=MIN_POINTS_PER_VOXEL,
    )

    filtered_pcd = filtered_pcd.voxel_down_sample(voxel_size=GRID_RESOLUTION)

    # --- 4. Visualize Filter Step ---
    print("\nVisualizing original vs. downsampled (Press 'Q' to close)")
    o3d.visualization.draw_geometries([demo_pcd], window_name="Original")
    o3d.visualization.draw_geometries([filtered_pcd], window_name="Voxel Density Filtered")

    # --- 5. Run the 3D Region-Growing algorithm ---
    NORMAL_ESTIMATION_RADIUS = 0.1
    NORMAL_ESTIMATION_NN = 30
    NEIGHBOR_SEARCH_RADIUS = 0.2
    NORMAL_ANGLE_THRESHOLD_DEG = 10.0  # Key tuning parameter
    SEED_SELECTION_METHOD = "origin"  # Use new "smart" origin seed
    ORIGIN_SEARCH_RADIUS = 1.5  # 1.5m radius around (0,0)
    SEED_MAX_GROUND_ANGLE_DEG = 30.0  # Max angle for a point to be a seed

    gnd_pcd, obs_pcd = region_growing_pcd(
        filtered_pcd,  # IMPORTANT: Use the downsampled cloud
        normal_estimation_radius=NORMAL_ESTIMATION_RADIUS,
        normal_estimation_nn=NORMAL_ESTIMATION_NN,
        neighbor_search_radius=NEIGHBOR_SEARCH_RADIUS,
        normal_angle_threshold_deg=NORMAL_ANGLE_THRESHOLD_DEG,
        seed_selection_method=SEED_SELECTION_METHOD,
        origin_search_radius=ORIGIN_SEARCH_RADIUS,
        seed_max_ground_angle_deg=SEED_MAX_GROUND_ANGLE_DEG,
    )

    # --- 6. Visualize the 3D classified result ---
    if obs_pcd and gnd_pcd:
        print("\nVisualizing classified 3D points (Green=Ground, Red=Obstacle)...")
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        o3d.visualization.draw_geometries(
            [gnd_pcd, obs_pcd, coord_frame], window_name="Classified Points (3D Region Growing)"
        )
    else:
        print("Error: 3D Segmentation failed. Exiting.")
        sys.exit(1)

    # --- 7. Project to 2D Grid, Smooth, and Save ---

    grid, origin, shape = create_occupancy_grid_from_pcds(gnd_pcd, obs_pcd, GRID_RESOLUTION)

    if grid is not None:
        gap_size_meters = 0.2
        gap_size_pixels = int(gap_size_meters / GRID_RESOLUTION)
        SMOOTHING_KERNEL_SIZE = gap_size_pixels
        if SMOOTHING_KERNEL_SIZE % 2 == 0:
            SMOOTHING_KERNEL_SIZE += 1

        grid = smooth_free_space(grid, kernel_size=SMOOTHING_KERNEL_SIZE)

        save_occupancy(grid, GRID_RESOLUTION, grid_path, pcd_name, origin)

        print("\nVisualizing final 2D Occupancy Grid...")
        plt.figure(figsize=(10, 10))
        extent = [origin[0], origin[0] + shape[1] * GRID_RESOLUTION, origin[1], origin[1] + shape[0] * GRID_RESOLUTION]

        plt.imshow(grid, cmap="gray_r", extent=extent, vmin=0.0, vmax=1.0)
        plt.title(f"2D Occupancy Grid (Resolution: {GRID_RESOLUTION}m)")
        plt.xlabel("X Coordinate (meters)")
        plt.ylabel("Y Coordinate (meters)")
        plt.gca().set_aspect("equal", adjustable="box")
        plt.show()
    else:
        print("Error: 2D Grid Generation failed.")
