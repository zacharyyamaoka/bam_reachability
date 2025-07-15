# BAM
from bam_reachability.generators import TablePoseGenerator
from bam_reachability.utils.math_utils import get_matrix

from bam_reachability.reachability_map import ReachabilityMap, make_map_path
from bam_reachability.kin_wrapper import UR5eOffsetWristKinWrapper
from bam_reachability.visualization import Open3DMapViewer, MeshcatMapViewer, colorize_reachability, colorize_inconsistency


# PYTHON
import numpy as np



K = UR5eOffsetWristKinWrapper()

map = ReachabilityMap.load("/home/bam/python_ws/bam_reachability/maps/ur/ur5e/ur5e_table_1.2x1.2x1.0_0.10_90x45x360_15_jul_2025.pkl")

colors = colorize_reachability(map, show_histogram=False)

meshcat_viewer = MeshcatMapViewer.from_xacro(map, colors, K.xacro_path, K.mesh_directory)
meshcat_viewer.run()
