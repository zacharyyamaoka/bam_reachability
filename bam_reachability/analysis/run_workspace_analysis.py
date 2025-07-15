#!/usr/bin/env python3

from bam_reachability.kin_wrapper import KinWrapper
from bam_reachability.generators import TablePoseGenerator, visualize_frames
from bam_reachability.utils.math_utils import get_matrix
import numpy as np
from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.visualization import colorize_reachability, colorize_inconsistency, Open3DMapViewer

# Load Kinematics

# Create Generic Map

# Create Hemisphere Map

class PointSpace():
    def __init__(self, points: np.ndarray):
        pass

    def sample(self):
        pass

    def to_stl(self):
        pass

    def to_meshcat(self):
        pass

    def to_bbox_aligned(self):
        pass

    def to_bbox_rotated(self):
        pass

    def to_sphere(self):
        pass

    def to_cylinder(self):
        pass

class RobotWorkspace():
    def __init__(self, dexterous_map: ReachabilityMap, hemisphere_map: ReachabilityMap, four_dof_map: ReachabilityMap):
        pass

    def sample_reachable_workspace(self):
        pass

    def sample_dexterous_workspace(self):
        pass

    def sample_hemisphere_workspace(self):
        pass

    def sample_four_dof_workspace(self):
        pass


def run_workspace_analysis(K: KinWrapper, table_pose_matrix: np.ndarray, table_scale=(1.25, 1.25, 1), xyz_step=0.1, viz=False, verbose=True):

    dexterous_ws_generator = TablePoseGenerator(
        pose_matrix = table_pose_matrix,
        scale = table_scale,
        xyz_step = xyz_step,
        hemisphere_angle = np.deg2rad(180), 
        view_step = np.deg2rad(45),
        rotation_step = np.deg2rad(360), # 360 for no rotation
        )

    dex_positions, dex_orientations = dexterous_ws_generator.generate(verbose=verbose)

    if viz:
        visualize_frames(dex_orientations, only_z=True)

    hemisphere_ws_generator = TablePoseGenerator(
        pose_matrix = table_pose_matrix,
        scale = table_scale,
        xyz_step = xyz_step,
        hemisphere_angle = np.deg2rad(85), # being perfectly aligned with 90, can cause issues...
        view_step = np.deg2rad(85/2),
        rotation_step = np.deg2rad(360), # 360 for no rotation
        )

    hemi_positions, hemi_orientations = hemisphere_ws_generator.generate(verbose=verbose)

    if viz:
        visualize_frames(hemi_orientations, only_z=True)

    dex_map = ReachabilityMap(dex_positions, dex_orientations, K.IK, K.FK)
    dex_map.calculate_ik_fk_map(verbose=verbose)

    colors = colorize_reachability(dex_map, show_histogram=True)
    Open3DMapViewer(dex_map.positions, colors=colors, hide_alpha=False).run()

    inconsistency_colors = colorize_inconsistency(dex_map, show_histogram=True)
    Open3DMapViewer(dex_map.positions, colors=inconsistency_colors).run()



    hemi_map = ReachabilityMap(hemi_positions, hemi_orientations, K.IK, K.FK)
    hemi_map.calculate_ik_fk_map(verbose=verbose)


    # Mesh cast viewer.... hmm that can likely come after tbh? or mabye not if its very intergral I gues...

if __name__ == "__main__":
    from bam_reachability.kin_wrapper import MockKinWrapper, UR5KinWrapper, UR3KinWrapper
    K = UR3KinWrapper()
    run_workspace_analysis(K, get_matrix(([0, 0, 0], [0, 0, 0])), xyz_step=0.25)