# BAM
from bam_reachability.generators import TablePoseGenerator
from bam_reachability.utils.math_utils import get_matrix

from bam_reachability.reachability_map import ReachabilityMap, make_map_path
from bam_reachability.kin_wrapper import MockKinWrapper
from bam_reachability.visualization.open3d_map_viewer import Open3DMapViewer
from bam_reachability.visualization.colorize_map import colorize_reachability, colorize_inconsistency

# PYTHON
import numpy as np


# 1. Generate Frames

generator = TablePoseGenerator(
    pose_matrix = get_matrix(([0, 0, 0], [0, 0, 0])),
    scale = (1.25, 1.25, 1),
    xyz_step = 0.1,
    hemisphere_angle = np.deg2rad(90),
    view_step = np.deg2rad(45),
    rotation_step = np.deg2rad(360),
    )

# generator = PlacePoseGenerator()

positions, orientations = generator.generate(verbose=False)

# 2. Create IK/FK functions
K = MockKinWrapper(L1=0.25, L2=0.25)

# 3. Create Map and save result
map = ReachabilityMap(positions, orientations, K.IK, K.FK)
map.calculate_ik_fk_map()


print(f"Step size should be some multiple of the xyz_step {generator.xyz_step}")
colors = colorize_reachability(map, show_histogram=True)
Open3DMapViewer(map.positions, step=0.2, colors=colors, hide_alpha=False).run()
Open3DMapViewer(map.positions, step=0.2, colors=colors, hide_alpha=True).run()

inconsistency_colors = colorize_inconsistency(map, show_histogram=True)
Open3DMapViewer(map.positions, colors=inconsistency_colors).run()
