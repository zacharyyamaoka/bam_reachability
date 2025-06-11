import numpy as np
import open3d as o3d

def rectangular_generator(scale=(1.0, 1.0, 1.0), step=0.1):
    x_range = np.arange(-scale[0] / 2, scale[0] / 2 + step, step)
    y_range = np.arange(-scale[1] / 2, scale[1] / 2 + step, step)
    z_range = np.arange(-scale[2] / 2, scale[2] / 2 + step, step)
    X, Y, Z = np.meshgrid(x_range, y_range, z_range, indexing='ij')
    points = np.vstack((X.ravel(), Y.ravel(), Z.ravel())).T
    return points

def mask_sphere(points, diameter=1.0, sign=1):
    """
        sign: 1 to return points inside, -1 to return points outside
    """
    radius = diameter / 2
    distances = np.linalg.norm(points, axis=1)
    if sign >= 0:
        return points[distances <= radius]
    else:
        return points[distances > radius]

def spherical_generator(diameter=1.0, step=0.1, sign=1):
    points = rectangular_generator(scale=(diameter, diameter, diameter), step=step)
    return mask_sphere(points, diameter=diameter, sign=sign)

def donut_generator(inner_diameter=0.1, outer_diameter=1.0, step=0.1):
    all_points = rectangular_generator(scale=(outer_diameter, outer_diameter, outer_diameter), step=step)
    outer_masked = mask_sphere(all_points, diameter=outer_diameter, sign=1)
    donut_points = mask_sphere(outer_masked, diameter=inner_diameter, sign=-1)
    return donut_points

def visualize_points(points: np.ndarray, axis_length=0.1):
    """
        points: np.ndarray (X, 3)
        axis_length: length of axis arrow in meters
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_length)
    o3d.visualization.draw_geometries([pcd, frame])

# Example usage:
if __name__ == "__main__":
    sphere = spherical_generator(diameter=1.0, step=0.05)
    visualize_points(sphere)

    donut = donut_generator(inner_diameter=0.4, outer_diameter=1.0, step=0.05)
    visualize_points(donut)

    rect = rectangular_generator(scale=(0.5, 0.5, 0.5), step=0.05)
    visualize_points(rect)
