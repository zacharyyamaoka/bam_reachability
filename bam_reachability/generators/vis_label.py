import open3d as o3d
import numpy as np

# COPIED FROM https://github.com/GouMinghao/rgb_matters/blob/main/rgbd_graspnet/data/utils/vis_label.py

def create_mesh_box(width, height, depth, dx=0, dy=0, dz=0):
    box = o3d.geometry.TriangleMesh()
    vertices = np.array(
        [
            [0, 0, 0],
            [width, 0, 0],
            [0, 0, depth],
            [width, 0, depth],
            [0, height, 0],
            [width, height, 0],
            [0, height, depth],
            [width, height, depth],
        ]
    )
    vertices[:, 0] += dx
    vertices[:, 1] += dy
    vertices[:, 2] += dz
    triangles = np.array(
        [
            [4, 7, 5],
            [4, 6, 7],
            [0, 2, 4],
            [2, 6, 4],
            [0, 1, 2],
            [1, 3, 2],
            [1, 5, 7],
            [1, 7, 3],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 1],
            [1, 4, 5],
        ]
    )
    box.vertices = o3d.utility.Vector3dVector(vertices)
    box.triangles = o3d.utility.Vector3iVector(triangles)
    return box

def plot_gripper_pro_max(center, R, width, depth, score=1):
    """
    center: target point
    R: rotation matrix
    """
    height = 0.004
    finger_width = 0.004
    tail_length = 0.04
    depth_base = 0.02

    color_r = score  # red for high score
    color_b = 1 - score  # blue for low score
    color_g = 0
    left = create_mesh_box(depth + depth_base + finger_width, finger_width, height)
    right = create_mesh_box(depth + depth_base + finger_width, finger_width, height)
    bottom = create_mesh_box(finger_width, width, height)
    tail = create_mesh_box(tail_length, finger_width, height)

    left_points = np.array(left.vertices)
    left_triangles = np.array(left.triangles)
    left_points[:, 0] -= depth_base + finger_width
    left_points[:, 1] -= width / 2 + finger_width
    left_points[:, 2] -= height / 2

    right_points = np.array(right.vertices)
    right_triangles = np.array(right.triangles) + 8
    right_points[:, 0] -= depth_base + finger_width
    right_points[:, 1] += width / 2
    right_points[:, 2] -= height / 2

    bottom_points = np.array(bottom.vertices)
    bottom_triangles = np.array(bottom.triangles) + 16
    bottom_points[:, 0] -= finger_width + depth_base
    bottom_points[:, 1] -= width / 2
    bottom_points[:, 2] -= height / 2

    tail_points = np.array(tail.vertices)
    tail_triangles = np.array(tail.triangles) + 24
    tail_points[:, 0] -= tail_length + finger_width + depth_base
    tail_points[:, 1] -= finger_width / 2
    tail_points[:, 2] -= height / 2

    vertices = np.concatenate(
        [left_points, right_points, bottom_points, tail_points], axis=0
    )
    vertices = np.dot(R, vertices.T).T + center
    triangles = np.concatenate(
        [left_triangles, right_triangles, bottom_triangles, tail_triangles], axis=0
    )
    colors = np.array([[color_r, color_g, color_b] for _ in range(len(vertices))])

    gripper = o3d.geometry.TriangleMesh()
    gripper.vertices = o3d.utility.Vector3dVector(vertices)
    gripper.triangles = o3d.utility.Vector3iVector(triangles)
    gripper.vertex_colors = o3d.utility.Vector3dVector(colors)
    return gripper