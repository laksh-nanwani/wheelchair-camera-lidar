import numpy as np
import open3d as o3d
from matplotlib.pyplot import imsave
import yaml
import os
import math

if __name__ == "__main__":
    map_name = "himalaya+kcis"
    pcd_path = f"/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/PCD/{map_name}.pcd"
    out_path = "/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO/maps"

    z_range = [-0.9, -0.1]
    res = 0.05

    print("Loading PCD")
    pcd = o3d.io.read_point_cloud(pcd_path)
    # o3d.visualization.draw_geometries([pcd])
    # T = np.identity(3)
    # theta = -3.0 * math.pi / 180.0  # Convert degrees to radians
    # T[0, 0] = np.cos(theta)
    # T[0, 2] = np.sin(theta)
    # T[2, 2] = np.cos(theta)
    # T[2, 0] = -np.sin(theta)

    # pcd.rotate(T, center=(0, 0, 0))
    print("PCD loaded")
    pcd = np.array(pcd.points)

    sorted_indices = np.argsort(pcd[:, 2])
    pcd = pcd[sorted_indices]
    print("Sorted PCD based on Z values")

    # mask = np.where((pcd[:, 2] > z_range[0]) & (pcd[:, :2] < z_range[1]))
    # filtered_pcd = pcd[mask]

    x_min = np.min(pcd[:, 0])
    x_max = np.max(pcd[:, 0])
    y_min = np.min(pcd[:, 1])
    y_max = np.max(pcd[:, 1])
    # z_min = np.min(pcd[:, 2])
    # z_max = np.max(pcd[:, 2])
    # print(z_min, z_max, pcd.shape)
    # exit()

    w = int((x_max - x_min) / res) + 1
    h = int((y_max - y_min) / res) + 1

    grid = np.full((h, w), 0.5)

    for p in pcd:
        x_grid = int((p[0] - x_min) / res)
        y_grid = h - 1 - int((p[1] - y_min) / res)

        if p[2] < z_range[0]:
            grid[y_grid, x_grid] = 1

        elif p[2] >= z_range[0] and p[2] <= z_range[1]:
            grid[y_grid, x_grid] = 0

    grid = (grid * 255).astype(int)

    map_dict = {
        "image": f"{map_name}.png",
        "resolution": res,
        "origin": [float(x_min), float(y_min), 0.0],
        "occupied_thresh": 0.6,
        "free_thresh": 0.3,
        "negate": 0,
    }

    imsave(os.path.join(out_path, f"{map_name}.png"), grid, cmap="gray")

    with open(os.path.join(out_path, f"{map_name}.yaml"), "w") as file:
        yaml.dump(map_dict, file, default_flow_style=None)

    print("Saved map to", out_path)
