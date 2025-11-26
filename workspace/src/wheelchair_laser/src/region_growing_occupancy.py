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


def filter_by_statistical_outlier(pcd, nb_neighbors, std_ratio):
    """
    Removes sparse outlier points (like 'ghost' trails) using
    Statistical Outlier Removal (SOR).
    ... (function contents unchanged) ...
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


# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
# --- NEW REGION-GROWING PATCHWORK ALGORITHM ---
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---


def get_z_from_model(model_tuple, x, y):
    """Helper to get Z value for a given (x,y) from a patch model."""
    model_type, model_data = model_tuple
    if model_type == "plane":
        a, b, c, d = model_data
        if c == 0:
            c = 1e-6  # Avoid division by zero
        return -(a * x + b * y + d) / c
    else:  # model_type == "height"
        return model_data


def are_patches_compatible(
    model1_tuple,
    model2_tuple,
    patch_coords1,
    patch_coords2,
    patch_size,
    max_angle_rad,
    max_height_diff,
):
    """
    Checks if two adjacent ground patches are compatible based on
    your rules (angle and height).
    """
    model1_type, model1_data = model1_tuple
    model2_type, model2_data = model2_tuple

    # 1. Angle Check (Your 6-degree rule)
    # Only check angles if both are planes
    if model1_type == "plane" and model2_type == "plane":
        normal1 = model1_data[:3]
        normal2 = model2_data[:3]

        # Handle opposing normals
        dot_product = abs(np.dot(normal1, normal2))
        # Clamp to 1.0 to avoid precision errors with arccos
        dot_product = min(1.0, dot_product)

        angle_rad = np.arccos(dot_product)
        if angle_rad > max_angle_rad:
            return False  # Angle difference is too large

    # 2. Height Check (Your 10cm rule)
    # Check height at the midpoint of their shared boundary
    patch_center1_x = patch_coords1[0] * patch_size + patch_size / 2
    patch_center1_y = patch_coords1[1] * patch_size + patch_size / 2

    patch_center2_x = patch_coords2[0] * patch_size + patch_size / 2
    patch_center2_y = patch_coords2[1] * patch_size + patch_size / 2

    # Midpoint of the boundary between the two patches
    boundary_x = (patch_center1_x + patch_center2_x) / 2
    boundary_y = (patch_center1_y + patch_center2_y) / 2

    z1_at_boundary = get_z_from_model(model1_tuple, boundary_x, boundary_y)
    z2_at_boundary = get_z_from_model(model2_tuple, boundary_x, boundary_y)

    if abs(z1_at_boundary - z2_at_boundary) > max_height_diff:
        return False  # Height jump is too large (e.g., table, stair)

    # If both checks pass
    return True


def region_growing_patchwork(
    pcd,
    patch_size=1.0,
    grid_resolution=0.1,
    ground_threshold=0.15,
    max_obstacle_height=2.0,
    ransac_dist_threshold=0.1,
    ransac_min_points=30,
    max_ground_angle_deg=30.0,
    max_ground_angle_diff_deg=6.0,
    max_ground_height_diff=0.1,
    seed_patch_coords=(0, 0),
):
    """
    Converts a 3D point cloud to a 2D occupancy grid using a
    Region-Growing Patchwork method.

    This method is robust to ramps, stairs, and disconnected surfaces
    (like tables).
    """

    print("Starting Region-Growing Patchwork Occupancy Grid Generation...")
    start_time = time.time()

    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        print("Error: Empty point cloud.")
        return None, None, None, None, None

    # --- 1. Define Grid Boundaries (ROS/Image convention) ---
    x_min, y_min, _ = np.min(points, axis=0)
    x_max, y_max, _ = np.max(points, axis=0)

    grid_cols = int((x_max - x_min) / grid_resolution) + 1
    grid_rows = int((y_max - y_min) / grid_resolution) + 1
    grid_shape = (grid_rows, grid_cols)
    grid_origin_xy = (x_min, y_min)

    if grid_rows == 0 or grid_cols == 0:
        print("Error: Invalid grid dimensions.")
        return None, None, None, None, None

    occupancy_grid = np.full(grid_shape, 0.5, dtype=np.float32)

    # --- 2. Pass 1: Bin points into patches ---
    print(f"Pass 1: Binning {points.shape[0]} points into {patch_size}m patches...")
    patches = defaultdict(list)
    point_to_patch_map = {}  # Store (px, py) for each point index
    for i in range(points.shape[0]):
        patch_x = int(np.floor(points[i, 0] / patch_size))
        patch_y = int(np.floor(points[i, 1] / patch_size))
        patch_coords = (patch_x, patch_y)
        patches[patch_coords].append(i)  # Store point index
        point_to_patch_map[i] = patch_coords

    # --- 3. Pass 2: Build Local Ground Model for ALL patches ---
    # This creates the "graph" of all potential ground planes
    print(f"Pass 2: Building local ground model for {len(patches)} patches...")
    ground_model = {}  # Stores (model_type, data)
    z_axis = np.array([0.0, 0.0, 1.0])
    max_angle_rad = np.deg2rad(max_ground_angle_deg)

    for patch_coords, indices in patches.items():
        patch_points = points[indices]

        if patch_points.shape[0] < ransac_min_points:
            if patch_points.shape[0] > 0:
                z_fallback = np.median(patch_points[:, 2])
                ground_model[patch_coords] = ("height", z_fallback)
            continue

        patch_pcd = o3d.geometry.PointCloud()
        patch_pcd.points = o3d.utility.Vector3dVector(patch_points)

        try:
            plane_model, inliers = patch_pcd.segment_plane(
                distance_threshold=ransac_dist_threshold, ransac_n=3, num_iterations=100
            )

            normal = plane_model[:3]
            normal_norm = np.linalg.norm(normal)
            if normal_norm == 0:
                continue

            normal = normal / normal_norm
            dot_product = abs(np.dot(normal, z_axis))
            dot_product = min(1.0, dot_product)  # Clamp
            angle_rad = np.arccos(dot_product)

            if abs(angle_rad) < max_angle_rad:
                ground_model[patch_coords] = ("plane", plane_model)
            else:
                z_fallback = np.median(patch_points[inliers, 2])
                ground_model[patch_coords] = ("height", z_fallback)

        except Exception as e:
            if patch_points.shape[0] > 0:
                z_fallback = np.median(patch_points[:, 2])
                ground_model[patch_coords] = ("height", z_fallback)

    # --- 4. Pass 3: Region Growing (BFS) to find ONE connected ground ---
    print("Pass 3: Performing region growing to find main ground...")
    main_ground_patches = set()
    visited_patches = set()
    queue = deque()

    # Find the closest patch to the seed coordinates that has a model
    if seed_patch_coords not in ground_model:
        # Seed patch is empty, find the nearest one with a model
        min_dist = float("inf")
        best_seed = None
        for patch_coords in ground_model.keys():
            dist = (patch_coords[0] - seed_patch_coords[0]) ** 2 + (patch_coords[1] - seed_patch_coords[1]) ** 2
            if dist < min_dist:
                min_dist = dist
                best_seed = patch_coords

        if best_seed:
            seed_patch_coords = best_seed
            print(f"  Seed (0,0) empty. Starting growth from nearest patch: {seed_patch_coords}")
        else:
            print("Error: No valid patches found. Aborting.")
            return None, None, None, None, None

    # Start the Breadth-First Search (BFS)
    queue.append(seed_patch_coords)
    visited_patches.add(seed_patch_coords)
    main_ground_patches.add(seed_patch_coords)

    max_angle_diff_rad = np.deg2rad(max_ground_angle_diff_deg)
    neighbors_offsets = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # 4-way neighbors

    while queue:
        current_patch_coords = queue.popleft()

        # Check if patch has a valid model, skip if not
        if current_patch_coords not in ground_model:
            continue

        current_model_tuple = ground_model[current_patch_coords]

        for dx, dy in neighbors_offsets:
            neighbor_patch_coords = (current_patch_coords[0] + dx, current_patch_coords[1] + dy)

            if neighbor_patch_coords in visited_patches:
                continue

            visited_patches.add(neighbor_patch_coords)

            if neighbor_patch_coords not in ground_model:
                continue  # This neighbor patch has no model

            neighbor_model_tuple = ground_model[neighbor_patch_coords]

            # Run your compatibility check!
            if are_patches_compatible(
                current_model_tuple,
                neighbor_model_tuple,
                current_patch_coords,
                neighbor_patch_coords,
                patch_size,
                max_angle_diff_rad,
                max_ground_height_diff,
            ):
                # This neighbor is compatible! Add it to the ground.
                main_ground_patches.add(neighbor_patch_coords)
                queue.append(neighbor_patch_coords)

    print(f"  Found {len(main_ground_patches)} connected ground patches.")

    # --- 5. Pass 4: Classify all points and build occupancy grid ---
    print("Pass 4: Classifying points and building grid...")
    obstacle_indices = []
    ground_indices = []

    for i in range(points.shape[0]):
        point = points[i]
        x, y, z = point

        # --- START FIX: O(N) Classification ---
        # This variable will track the status of the *current* point
        point_status = "ignored"
        # --- END FIX ---

        patch_coords = point_to_patch_map.get(i)
        if patch_coords is None:
            continue  # Should not happen, but as a safeguard

        is_on_main_ground = patch_coords in main_ground_patches

        if is_on_main_ground:
            # This point is on the connected ground. Classify it.
            model_tuple = ground_model[patch_coords]
            z_ground = get_z_from_model(model_tuple, x, y)
            z_relative = z - z_ground

            # --- Classification ---
            if ground_threshold <= z_relative < max_obstacle_height:
                # OBSTACLE on top of the main ground
                obstacle_indices.append(i)
                point_status = "obstacle"  # <-- FIX
            elif abs(z_relative) < ground_threshold:
                # GROUND point
                ground_indices.append(i)
                point_status = "ground"  # <-- FIX
            # else: Point is too high/low (ignored)

        else:
            # This point is NOT on the main ground (e.g., table, stair, wall).
            # It's an OBSTACLE.

            # Only add if it's within a reasonable height range
            # to avoid adding ceilings etc.
            # We need a ground-truth... let's just use the seed's ground
            seed_model = ground_model[seed_patch_coords]
            z_seed_ground = get_z_from_model(seed_model, x, y)
            z_relative_to_seed = z - z_seed_ground

            if 0.0 < z_relative_to_seed < max_obstacle_height * 2.0:  # Generous range
                obstacle_indices.append(i)
                point_status = "obstacle"  # <-- FIX

        # --- START FIX: O(N) Grid Update ---
        # This check is now O(1) instead of O(N)
        # We only project points that are classified as ground or obstacle

        if point_status != "ignored":
            # Project to 2D occupancy grid
            grid_c = int((x - x_min) / grid_resolution)
            grid_r = (grid_rows - 1) - int((y - y_min) / grid_resolution)

            if 0 <= grid_r < grid_rows and 0 <= grid_c < grid_cols:
                if point_status == "obstacle":
                    occupancy_grid[grid_r, grid_c] = 1.0  # Mark as occupied

                elif point_status == "ground":
                    # Only mark as 'free' (0.0) if not already 'occupied' (1.0)
                    if occupancy_grid[grid_r, grid_c] != 1.0:
                        occupancy_grid[grid_r, grid_c] = 0.0
        # --- END FIX ---

    # --- 6. Create final outputs ---
    print("Finalizing outputs...")
    # We can't use the 'points' array directly if it was modified
    # Re-build from the original 'points' array
    obstacle_points = np.asarray(pcd.points)[obstacle_indices]
    ground_points = np.asarray(pcd.points)[ground_indices]

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
    ... (function contents unchanged) ...
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
    ... (function contents unchanged) ...
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

if __name__ == "__main__":
    # --- 1. Load Point Cloud from file ---
    pcd_name = "lab_and_himalaya"
    pcd_path = f"/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/PCD/{pcd_name}.pcd"
    grid_path = "/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/maps_region/"

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

    # --- --- --- --- --- --- --- --- --- --- --- ---
    # --- NEW STEP 1.5: Pre-Downsample Point Cloud  ---
    # --- --- --- --- --- --- --- --- --- --- --- ---
    # This is the single biggest speedup.
    # It reduces the number of points all other functions must process.

    # --- 2. Filter dynamic object trails ---
    # Choose one of the filters below

    # --- Option A: Voxel Density Filter ---
    DENSITY_VOXEL_SIZE = 0.1
    MIN_POINTS_PER_VOXEL = 10
    filtered_pcd = filter_by_voxel_density(
        demo_pcd,  # <-- IMPORTANT: Use the downsampled cloud
        voxel_size=DENSITY_VOXEL_SIZE,
        min_points_per_voxel=MIN_POINTS_PER_VOXEL,
    )

    # --- Option B: Statistical Outlier Removal ---
    # SOR_NEIGHBORS = 30
    # SOR_STD_RATIO = 2.0
    # filtered_pcd = filter_by_statistical_outlier(
    #     downsampled_pcd, # <-- IMPORTANT: Use the downsampled cloud
    #     nb_neighbors=SOR_NEIGHBORS,
    #     std_ratio=SOR_STD_RATIO
    # )

    # --- 3. Visualize Filter Step ---
    print("\nVisualizing original vs. filtered (Press 'Q' to close)")
    # Show the downsampled one for a fair comparison
    o3d.visualization.draw_geometries([demo_pcd], window_name="Original")
    o3d.visualization.draw_geometries([filtered_pcd], window_name="Filtered")

    # --- 4. Run the Region-Growing Patchwork algorithm ---

    # --- General Hyperparameters ---
    PATCH_SIZE = 0.5  # 0.5m x 0.5m patches
    GRID_RESOLUTION = 0.05  # 5cm grid cells
    GROUND_THRESHOLD = 0.05  # 5cm tolerance for ground
    MAX_OBSTACLE_HEIGHT = 1.5  # Ignore points > 1.5m above ground
    RANSAC_DIST_THRESHOLD = 0.05  # 5cm RANSAC tolerance
    RANSAC_MIN_POINTS = 30  # Min points in a patch for RANSAC
    MAX_GROUND_ANGLE_DEG = 10  # Max angle for a plane to be "ground" (vs. a wall)

    # --- NEW Region Growing Hyperparameters ---
    # Your 6-degree rule for ramps
    MAX_GROUND_ANGLE_DIFF_DEG = 6.0
    # Your 10cm rule for stairs/tables
    MAX_GROUND_HEIGHT_DIFF = 0.02
    # Where to start growing from (patch coordinates)
    SEED_PATCH_COORDS = (0, 0)

    grid, obs_pcd, gnd_pcd, origin, shape = region_growing_patchwork(
        filtered_pcd,
        patch_size=PATCH_SIZE,
        grid_resolution=GRID_RESOLUTION,
        ground_threshold=GROUND_THRESHOLD,
        max_obstacle_height=MAX_OBSTACLE_HEIGHT,
        ransac_dist_threshold=RANSAC_DIST_THRESHOLD,
        ransac_min_points=RANSAC_MIN_POINTS,
        max_ground_angle_deg=MAX_GROUND_ANGLE_DEG,
        max_ground_angle_diff_deg=MAX_GROUND_ANGLE_DIFF_DEG,
        max_ground_height_diff=MAX_GROUND_HEIGHT_DIFF,
        seed_patch_coords=SEED_PATCH_COORDS,
    )

    # --- 5. Visualize the 3D classified result ---
    if obs_pcd and gnd_pcd:
        print("\nVisualizing classified 3D points (Green=Ground, Red=Obstacle)...")
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        o3d.visualization.draw_geometries(
            [gnd_pcd, obs_pcd, coord_frame], window_name="Classified Points (Region Growing)"
        )

    # --- 6. Smooth and Save the 2D Occupancy Grid ---
    if grid is not None:
        # Auto-calculate kernel size
        gap_size_pixels = int(PATCH_SIZE / GRID_RESOLUTION)
        SMOOTHING_KERNEL_SIZE = gap_size_pixels + 3
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
