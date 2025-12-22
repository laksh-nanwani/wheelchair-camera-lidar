import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import time
from collections import deque
import sys
import os
import yaml
from matplotlib.pyplot import imsave
from scipy import ndimage

# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
# --- 1. FILTERING UTILITIES ---
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---


def filter_by_voxel_density(pcd, voxel_size, min_points_per_voxel):
    """
    Removes sparse points (ghost trails) using 1D Voxel Hashing.
    (Optimized for speed: uses 1D arithmetic instead of 3D row sorting)
    """
    print("Starting Voxel Density Filter (Fast)...")
    start_time = time.time()

    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        return pcd

    # 1. Calculate Grid Indices (Int64 to prevent overflow)
    points_min = points.min(axis=0)
    voxel_indices = np.floor((points - points_min) / voxel_size).astype(np.int64)

    # 2. Flatten 3D indices to 1D (Hashing)
    # This avoids the slow np.unique(axis=0) which sorts rows
    max_idx = voxel_indices.max(axis=0) + 1

    # Strides for flattening: x + y*dX + z*dX*dY
    stride_x = 1
    stride_y = max_idx[0]
    stride_z = max_idx[0] * max_idx[1]

    flat_indices = voxel_indices[:, 0] * stride_x + voxel_indices[:, 1] * stride_y + voxel_indices[:, 2] * stride_z

    # 3. Unique Count on 1D Array (Very Fast)
    # We sort the flat indices to group identical voxels
    sort_idx = np.argsort(flat_indices)
    sorted_indices = flat_indices[sort_idx]

    # Find where the voxel index changes
    # 'flag' is True where the index changes
    flag = np.concatenate(([True], sorted_indices[1:] != sorted_indices[:-1]))

    # Get unique counts
    # This gives us the count for every unique voxel found
    unique_counts = np.diff(np.nonzero(np.r_[flag, True])[0])

    # 4. Filter
    # Identifying which UNIQUE voxels are dense enough
    dense_mask = unique_counts >= min_points_per_voxel

    # Now we need to map this back to the original points.
    # We expand the dense_mask back to the size of the sorted points
    # 'flag' marks the start of each new voxel group. We can use repeat/cumsum tricks,
    # but since we have the counts, we can simply repeat the mask.
    points_keep_mask_sorted = np.repeat(dense_mask, unique_counts)

    # Map back to original order using the inverse of sort_idx
    # Creating an empty boolean mask
    points_keep_mask = np.empty_like(points_keep_mask_sorted)
    points_keep_mask[sort_idx] = points_keep_mask_sorted

    keep_indices = np.where(points_keep_mask)[0]

    filtered_pcd = pcd.select_by_index(keep_indices)

    dt = time.time() - start_time
    removed = len(points) - len(keep_indices)
    print(f"  ...Filter done in {dt:.3f}s. Removed {removed} points ({removed / len(points) * 100:.1f}%).")

    return filtered_pcd


# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
# --- 2. FAST UTILITIES ---
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---


def get_z_from_plane_vectorized(plane_coeffs, x, y):
    """
    Vectorized calculation of Z from plane equation.
    plane_coeffs: (N, 4)
    x, y: (N,)
    """
    a, b, c, d = plane_coeffs[:, 0], plane_coeffs[:, 1], plane_coeffs[:, 2], plane_coeffs[:, 3]
    # Avoid div/0 with small epsilon
    c = np.where(np.abs(c) < 1e-6, 1e-6, c)
    return -(a * x + b * y + d) / c


def check_compatibility_vectorized(curr_data, nbr_data, curr_idx, nbr_idx, patch_size, max_angle, max_h_diff):
    """
    Fast check between two patches using array data.
    """
    # Unpack
    c_type, c_plane, c_z = curr_data
    n_type, n_plane, n_z = nbr_data

    # 1. Angle Check (Only if both are planes) (Type 1 = Plane)
    if c_type == 1 and n_type == 1:
        dot = abs(np.dot(c_plane[:3], n_plane[:3]))
        angle = np.arccos(min(1.0, dot))
        if angle > max_angle:
            return False

    # 2. Height Check at Boundary
    # Patch Centers
    cx, cy = (curr_idx * patch_size) + (patch_size / 2)
    nx, ny = (nbr_idx * patch_size) + (patch_size / 2)

    # Midpoint
    mx, my = (cx + nx) / 2, (cy + ny) / 2

    # Get Z at midpoint
    z1 = -(c_plane[0] * mx + c_plane[1] * my + c_plane[3]) / c_plane[2] if c_type == 1 else c_z
    z2 = -(n_plane[0] * mx + n_plane[1] * my + n_plane[3]) / n_plane[2] if n_type == 1 else n_z

    return abs(z1 - z2) < max_h_diff


# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
# --- 3. OPTIMIZED REGION GROWING ---
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---


def region_growing_patchwork_optimized(
    pcd,
    patch_size=1.0,
    grid_resolution=0.1,
    ground_threshold=0.15,
    max_obstacle_height=2.0,
    ransac_dist_threshold=0.1,
    ransac_min_points=30,
    max_ground_angle_deg=15.0,
    max_ground_angle_diff_deg=15.0,
    max_ground_height_diff=0.15,
    seed_patch_coords=(0, 0),
    rescue_disconnected_ground=True,
):
    print("\nStarting Optimized Region Growing...")
    start_time = time.time()

    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        return None, None, None, None, None

    # --- 1. Grid Initialization (Vectorized) ---
    x_min, y_min, _ = np.min(points, axis=0)
    x_max, y_max, _ = np.max(points, axis=0)

    grid_cols = int((x_max - x_min) / grid_resolution) + 1
    grid_rows = int((y_max - y_min) / grid_resolution) + 1

    # Patch indices
    patch_idx_x = np.floor((points[:, 0]) / patch_size).astype(int)
    patch_idx_y = np.floor((points[:, 1]) / patch_size).astype(int)

    min_px, min_py = patch_idx_x.min(), patch_idx_y.min()
    max_px, max_py = patch_idx_x.max(), patch_idx_y.max()

    p_rows = max_py - min_py + 1
    p_cols = max_px - min_px + 1

    grid_type = np.zeros((p_rows, p_cols), dtype=int)
    grid_plane = np.zeros((p_rows, p_cols, 4), dtype=float)
    grid_height = np.zeros((p_rows, p_cols), dtype=float)
    grid_valid = np.zeros((p_rows, p_cols), dtype=bool)

    # Shift indices to 0-based
    shifted_px = patch_idx_x - min_px
    shifted_py = patch_idx_y - min_py

    z_axis = np.array([0, 0, 1.0])
    max_angle_rad = np.deg2rad(max_ground_angle_deg)

    # Sort points to group them by patch
    sort_idx = np.lexsort((shifted_px, shifted_py))
    sorted_points = points[sort_idx]
    sorted_px = shifted_px[sort_idx]
    sorted_py = shifted_py[sort_idx]

    diff_mask = (sorted_px[1:] != sorted_px[:-1]) | (sorted_py[1:] != sorted_py[:-1])
    split_indices = np.flatnonzero(diff_mask) + 1
    patch_groups = np.split(sorted_points, split_indices)

    # coords stores [px, py] pairs
    patch_coords = np.split(np.column_stack((sorted_px, sorted_py)), split_indices)

    print(f"  Modeling {len(patch_groups)} patches...")

    # Modeling Loop
    for group, coords in zip(patch_groups, patch_coords):
        if len(group) < 1:
            continue

        # --- FIX IS HERE: Correct Unpacking ---
        px, py = coords[0]  # Was 'py, px', causing X/Y swap

        if len(group) < ransac_min_points:
            grid_type[py, px] = 2
            grid_height[py, px] = np.mean(group[:, 2])
            grid_valid[py, px] = True
            continue

        pcd_tmp = o3d.geometry.PointCloud()
        pcd_tmp.points = o3d.utility.Vector3dVector(group)
        try:
            plane, _ = pcd_tmp.segment_plane(ransac_dist_threshold, 3, 50)
            normal = plane[:3] / np.linalg.norm(plane[:3])
            angle = np.arccos(min(1.0, abs(np.dot(normal, z_axis))))

            if angle < max_angle_rad:
                grid_type[py, px] = 1
                grid_plane[py, px] = plane
            else:
                grid_type[py, px] = 2
                grid_height[py, px] = np.median(group[:, 2])
            grid_valid[py, px] = True
        except:
            pass

    # --- 2. Region Growing (Grid BFS) ---
    print("  Growing regions...")

    ground_mask_grid = np.zeros((p_rows, p_cols), dtype=bool)
    visited_grid = np.zeros((p_rows, p_cols), dtype=bool)

    seed_px, seed_py = seed_patch_coords[0] - min_px, seed_patch_coords[1] - min_py

    # Seed Validation
    if not (0 <= seed_px < p_cols and 0 <= seed_py < p_rows and grid_type[seed_py, seed_px] == 1):
        best_z = float("inf")
        found_seed = False
        # Search radius 5 around seed
        r_min, r_max = max(0, seed_py - 5), min(p_rows, seed_py + 6)
        c_min, c_max = max(0, seed_px - 5), min(p_cols, seed_px + 6)

        for r in range(r_min, r_max):
            for c in range(c_min, c_max):
                if grid_type[r, c] == 1:
                    cx_world = (c + min_px) * patch_size + patch_size / 2
                    cy_world = (r + min_py) * patch_size + patch_size / 2
                    pl = grid_plane[r, c]
                    z = -(pl[0] * cx_world + pl[1] * cy_world + pl[3]) / (pl[2] + 1e-6)
                    if z < best_z:
                        best_z = z
                        seed_px, seed_py = c, r
                        found_seed = True
        if not found_seed:
            print("  Warning: No valid seed found. Returning raw data.")
            # Fallback to prevent crash: use first valid patch or empty
            return None, None, None, None, None

    queue = deque([(seed_px, seed_py)])
    visited_grid[seed_py, seed_px] = True
    ground_mask_grid[seed_py, seed_px] = True

    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
    max_angle_diff = np.deg2rad(max_ground_angle_diff_deg)

    while queue:
        cx, cy = queue.popleft()

        c_data = (grid_type[cy, cx], grid_plane[cy, cx], grid_height[cy, cx])
        c_idx = np.array([cx + min_px, cy + min_py])

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy

            if 0 <= nx < p_cols and 0 <= ny < p_rows:
                if visited_grid[ny, nx] or not grid_valid[ny, nx]:
                    continue

                n_data = (grid_type[ny, nx], grid_plane[ny, nx], grid_height[ny, nx])
                n_idx = np.array([nx + min_px, ny + min_py])

                if check_compatibility_vectorized(
                    c_data, n_data, c_idx, n_idx, patch_size, max_angle_diff, max_ground_height_diff
                ):
                    visited_grid[ny, nx] = True
                    ground_mask_grid[ny, nx] = True
                    queue.append((nx, ny))

    # --- 3. Rescue Pass (Vectorized) ---
    if rescue_disconnected_ground and np.any(ground_mask_grid):
        print("  Rescuing disconnected patches...")
        g_rows, g_cols = np.where(ground_mask_grid)
        if len(g_rows) > 0:
            gx = (g_cols + min_px) * patch_size + patch_size / 2
            gy = (g_rows + min_py) * patch_size + patch_size / 2

            planes = grid_plane[g_rows, g_cols]
            gz = -(planes[:, 0] * gx + planes[:, 1] * gy + planes[:, 3]) / (planes[:, 2] + 1e-6)

            is_simple = grid_type[g_rows, g_cols] == 2
            gz[is_simple] = grid_height[g_rows, g_cols][is_simple]

            avg_ground_z = np.mean(gz)

            # Candidates: Valid, Not Visited, Is Plane
            candidate_mask = (grid_valid) & (~ground_mask_grid) & (grid_type == 1)
            c_rows, c_cols = np.where(candidate_mask)

            if len(c_rows) > 0:
                cx_world = (c_cols + min_px) * patch_size + patch_size / 2
                cy_world = (c_rows + min_py) * patch_size + patch_size / 2
                c_planes = grid_plane[c_rows, c_cols]
                c_z = -(c_planes[:, 0] * cx_world + c_planes[:, 1] * cy_world + c_planes[:, 3]) / (
                    c_planes[:, 2] + 1e-6
                )

                rescue_mask = np.abs(c_z - avg_ground_z) < 0.3

                rescued_rows = c_rows[rescue_mask]
                rescued_cols = c_cols[rescue_mask]
                ground_mask_grid[rescued_rows, rescued_cols] = True
                print(f"    Rescued {len(rescued_rows)} patches.")

    # --- 4. Point Classification (Vectorized) ---
    print("  Classifying points...")
    valid_points_mask = (shifted_px >= 0) & (shifted_px < p_cols) & (shifted_py >= 0) & (shifted_py < p_rows)

    p_x = shifted_px[valid_points_mask]
    p_y = shifted_py[valid_points_mask]
    points_valid = points[valid_points_mask]

    point_in_ground_patch = ground_mask_grid[p_y, p_x]

    p_planes = grid_plane[p_y, p_x]
    p_heights = grid_height[p_y, p_x]
    p_types = grid_type[p_y, p_x]

    p_plane_z = get_z_from_plane_vectorized(p_planes, points_valid[:, 0], points_valid[:, 1])
    ref_z = np.where(p_types == 1, p_plane_z, p_heights)

    # Fallback for non-ground points
    if "avg_ground_z" in locals():
        ref_z[~point_in_ground_patch] = avg_ground_z
    else:
        ref_z[~point_in_ground_patch] = 0.0

    z_diff = points_valid[:, 2] - ref_z

    is_ground = point_in_ground_patch & (np.abs(z_diff) < ground_threshold)
    is_obstacle = (~is_ground) & (z_diff < max_obstacle_height) & (z_diff > -5.0)
    is_obstacle = is_obstacle & (z_diff > ground_threshold)

    # --- 5. Generate Occupancy Grid ---
    print("  Generating Grid...")
    occ_grid = np.full((grid_rows, grid_cols), 0.5, dtype=np.float32)

    gx = ((points_valid[:, 0] - x_min) / grid_resolution).astype(int)
    gy = ((points_valid[:, 1] - y_min) / grid_resolution).astype(int)

    gy = (grid_rows - 1) - gy  # Invert Y for image

    g_mask = (gx >= 0) & (gx < grid_cols) & (gy >= 0) & (gy < grid_rows)

    gx = gx[g_mask]
    gy = gy[g_mask]
    is_obs_masked = is_obstacle[g_mask]
    is_gnd_masked = is_ground[g_mask]

    # Prioritize Obstacles
    occ_grid[gy[is_gnd_masked], gx[is_gnd_masked]] = 0.0
    occ_grid[gy[is_obs_masked], gx[is_obs_masked]] = 1.0

    # Output PCDs
    obs_indices = np.where(valid_points_mask)[0][is_obstacle]
    gnd_indices = np.where(valid_points_mask)[0][is_ground]

    obs_pcd = o3d.geometry.PointCloud()
    if len(obs_indices) > 0:
        obs_pcd.points = o3d.utility.Vector3dVector(points[obs_indices])
        obs_pcd.paint_uniform_color([1, 0, 0])

    gnd_pcd = o3d.geometry.PointCloud()
    if len(gnd_indices) > 0:
        gnd_pcd.points = o3d.utility.Vector3dVector(points[gnd_indices])
        gnd_pcd.paint_uniform_color([0, 0.6, 0.2])

    print(f"Done. Total time: {time.time() - start_time:.2f}s")
    return occ_grid, obs_pcd, gnd_pcd, (x_min, y_min), (grid_rows, grid_cols)


# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
# --- MAIN ---
# --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---


def smooth_free_space(grid, kernel_size=3):
    obs_mask = grid == 1.0
    free_mask = grid == 0.0
    filled = ndimage.binary_closing(free_mask, structure=np.ones((kernel_size, kernel_size)))
    smooth_grid = np.full(grid.shape, 0.5, dtype=np.float32)
    smooth_grid[filled] = 0.0
    smooth_grid[obs_mask] = 1.0
    return smooth_grid


def save_occupancy(grid, res, out_path, map_name, origin):
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid == 0.0] = 254
    img[grid == 1.0] = 0
    full_path = os.path.join(out_path, f"{map_name}.png")
    imsave(full_path, img, cmap="gray", vmin=0, vmax=255)

    yaml_dict = {
        "image": f"{map_name}.png",
        "resolution": res,
        "origin": [float(origin[0]), float(origin[1]), 0.0],
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "negate": 0,
    }
    with open(os.path.join(out_path, f"{map_name}.yaml"), "w") as f:
        yaml.dump(yaml_dict, f)
    print(f"Saved to {full_path}")


if __name__ == "__main__":
    PCD_FILE = "nexus_first_floor_with_stores"
    INPUT_PATH = f"/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/PCD/{PCD_FILE}.pcd"
    OUTPUT_DIR = "/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/maps_region_fast/"

    print(f"Loading Point Cloud: {INPUT_PATH}")
    pcd = o3d.io.read_point_cloud(INPUT_PATH)
    if not pcd.has_points():
        sys.exit(1)

    print(f"Loaded {pcd.points} points.")

    filtered_pcd = filter_by_voxel_density(pcd, 0.1, 10)

    # o3d.visualization.draw_geometries([pcd])
    # o3d.visualization.draw_geometries([filtered_pcd])

    grid, obs_pcd, gnd_pcd, origin, shape = region_growing_patchwork_optimized(
        filtered_pcd,
        patch_size=0.5,
        grid_resolution=0.05,
        ground_threshold=0.05,
        max_obstacle_height=1.6,
        max_ground_angle_deg=6.0,
        max_ground_angle_diff_deg=12.0,
        max_ground_height_diff=0.05,
        rescue_disconnected_ground=True,
    )

    if grid is not None:
        if not os.path.exists(OUTPUT_DIR):
            os.makedirs(OUTPUT_DIR)
        smooth_k = int(0.25 / 0.05) + 1
        if smooth_k % 2 == 0:
            smooth_k += 1
        final_grid = smooth_free_space(grid, kernel_size=smooth_k)
        save_occupancy(final_grid, 0.05, OUTPUT_DIR, PCD_FILE + "_fast", origin)

        # Visualization
        o3d.visualization.draw_geometries([gnd_pcd, obs_pcd])
