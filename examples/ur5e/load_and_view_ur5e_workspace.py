from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.visualization.colorize_map import colorize_reachability, colorize_inconsistency
from bam_reachability.kin_wrapper import UR5eOffsetWristKinWrapper
from bam_reachability.reachability_workspace import ReachabilityWorkspace
from bam_reachability.visualization.meshcat_workspace_viewer import MeshcatWorkspaceViewer

import numpy as np

map = ReachabilityMap.load("/home/bam/python_ws/bam_reachability/maps/ur/ur5e/ur5e_table_1.2x1.2x1.0_0.10_180x45x360_15_jul_2025.pkl")

table_pose_matrix = np.eye(4)
z_axis = -1 * table_pose_matrix[:3, 2]

workspace = ReachabilityWorkspace(map, z_axis)

K = UR5eOffsetWristKinWrapper()

viewer = MeshcatWorkspaceViewer.from_xacro(workspace, K.xacro_path, K.mesh_directory)
viewer.run()