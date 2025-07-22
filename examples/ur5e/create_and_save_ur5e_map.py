# BAM
from bam_reachability.generators import TablePoseGenerator
from bam_reachability.utils.math_utils import get_matrix

from bam_reachability.reachability_map import ReachabilityMap, make_map_path
from bam_reachability.kin_wrapper import UR5eOffsetWristKinWrapper
from bam_reachability.visualization import Open3DMapViewer, MeshcatMapViewer, colorize_reachability, colorize_inconsistency


# PYTHON
import numpy as np
import os

# 1. Generate Frames

generator = TablePoseGenerator(
    pose_matrix = get_matrix(([0, 0, 0], [0, 0, 0])),
    scale = (1.25, 1.25, 1),
    xyz_step = 0.1,
    hemisphere_angle = np.deg2rad(180),
    view_step = np.deg2rad(45),
    rotation_step = np.deg2rad(360),
    )

positions, orientations = generator.generate(verbose=True)

# 2. Create IK/FK functions
K = UR5eOffsetWristKinWrapper()

# 3. Create Map and save result
map = ReachabilityMap(positions, orientations, K.IK, K.FK)
map.calculate_ik_fk_map(verbose=True)

map_dir = "/home/bam/python_ws/bam_reachability/maps/ur/ur5e"
map_name = f"{K.name}_{generator.name}"
file_path = os.path.join(map_dir, map_name)
map.save(file_path)



