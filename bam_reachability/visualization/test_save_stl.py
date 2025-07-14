import open3d as o3d
import numpy as np

# Example: convex hull mesh of point cloud
points = np.random.rand(100, 3)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

hull, _ = pcd.compute_convex_hull()
hull = hull.simplify_vertex_clustering(voxel_size=0.001)

# Export to STL
hull.compute_vertex_normals()  # ✅ REQUIRED for STL export
o3d.io.write_triangle_mesh("convex_hull.stl", hull)