import numpy as np
import open3d as o3d

def table_workspace_generator(pose_matrix, scale=(1.0, 0.5, 0.2), step=0.1) -> np.ndarray:
    """
    Pose is for the center surface of table, should be with respect to the ik base_frame of robot
    Scale is center on +-x, +-y but just +z (as you cannot go into the table)
    """
    local_points = rectangular_generator(scale=(scale[0], scale[1], scale[2]), step=step)

    # Manually offset Z up so the box is from 0 to +scale[2] instead of centered
    local_points[:, 2] += scale[2] / 2.0

    # convert into vector with 1 so can dot product with transform matrix
    ones = np.ones((local_points.shape[0], 1))
    local_points_h = np.hstack((local_points, ones))
    world_points_h = (pose_matrix @ local_points_h.T).T
    return world_points_h[:, :3]


def rectangular_generator(scale=(1.0, 1.0, 1.0), step=0.1) -> np.ndarray:
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

def visualize_points(points: np.ndarray, axis_length=0.1, pose_matrix: np.ndarray = None):
    """
    Visualize a point cloud with coordinate frame(s).

    Args:
        points: (N, 3) numpy array of point cloud.
        axis_length: Size of the coordinate frame arrows.
        pose_matrix: Optional 4x4 transform to visualize an additional frame.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_length*2)

    geometries = [pcd, origin_frame]

    if pose_matrix is not None:
        pose_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_length)
        pose_frame.transform(pose_matrix)
        geometries.append(pose_frame)

    o3d.visualization.draw_geometries(geometries)

def visualize_convex_hull(points: np.ndarray, axis_length=0.1):
    """
    Visualize the convex hull of a set of 3D points.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    hull, _ = pcd.compute_convex_hull()
    hull = hull.simplify_quadric_decimation(target_number_of_triangles=100)
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color([1.0, 0.0, 0.0])  # Red lines for hull
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_length)
    o3d.visualization.draw_geometries([pcd, hull_ls, frame])
    hull.compute_vertex_normals()
    o3d.io.write_triangle_mesh("convex_hull.stl", hull)


# Example usage:
if __name__ == "__main__":

    from bam_reachability.utils import xyzrpy_to_matrix
    # sphere = spherical_generator(diameter=1.0, step=0.05)
    # visualize_points(sphere)
    # visualize_convex_hull(sphere)

    # donut = donut_generator(inner_diameter=0.4, outer_diameter=1.0, step=0.05)
    # visualize_points(donut)

    # rect = rectangular_generator(scale=(0.5, 0.5, 0.5), step=0.05)
    # visualize_points(rect)


    pose_matrix = xyzrpy_to_matrix([0,0,0],[0,0,np.pi/4])
    # Using a step of 0.1 makes it easy to count!
    points = table_workspace_generator(pose_matrix, scale=(1.0, 0.5, 0.2), step=0.1) 

    visualize_points(points, pose_matrix=pose_matrix)