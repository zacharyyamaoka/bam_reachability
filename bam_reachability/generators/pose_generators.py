#!/usr/bin/env python3

# BAM
from bam_reachability.utils import get_matrix, matrix_to_rpy
from bam_reachability.generators import table_point_generator, view_generator, visualize_points, visualize_frames

# PYTHON
import numpy as np
from typing import Tuple


class PlacePoseGenerator:
    def __init__(self,
                 arm_name = 'ur',
                 pose = ([0, 0.5, 0], [-0.3, 0, 0]),
                 scale = (1.0, 0.5, 0.2),
                 xyz_step = 0.1,
                 hemisphere_angle = np.deg2rad(85),
                 view_step = np.deg2rad(85 / 2),
                 rotation_step = np.deg2rad(360),
                 viz = False):
        self.arm_name = arm_name
        self.pose = pose
        self.scale = scale
        self.xyz_step = xyz_step
        self.hemisphere_angle = hemisphere_angle
        self.view_step = view_step
        self.rotation_step = rotation_step
        self.viz = viz

    def generate(self, viz=True) -> Tuple[np.ndarray, np.ndarray]:
        pose_matrix = get_matrix(self.pose)

        place_1 = [0.5, 0.25, 0.25]
        place_2 = [-0.5, 0.25, 0.25]

        positions = np.array([place_1, place_2])

        z_axis = -1 * np.array([0.1, 0.1, 1]) #todo edit this later on to align
        R_list = view_generator(
            inital_view= z_axis,
            hemisphere_angle=self.hemisphere_angle,
            view_step=self.view_step,
            rotation_step=self.rotation_step
        )
        orientations = np.array([matrix_to_rpy(R) for R in R_list])

        if viz:
            visualize_points(positions)
            visualize_frames(R_list, only_z=True)

        return positions, orientations

    @property
    def name(self) -> str:
        # Create a concise signature with key parameters
        xyz_str = f"{self.xyz_step:.2f}"
        scale_str = "x".join([f"{s:.1f}" for s in self.scale])
        angles_str = f"{np.rad2deg(self.hemisphere_angle):.0f}x{np.rad2deg(self.view_step):.0f}x{np.rad2deg(self.rotation_step):.0f}"
        return f"place"

             

class TablePoseGenerator:
    def __init__(self,
                 pose=([0, 0.5, 0], [-0.3, 0, 0]),
                 scale=(1.0, 0.5, 0.2),
                 xyz_step=0.1,
                 hemisphere_angle=np.deg2rad(85),
                 view_step=np.deg2rad(85 / 2),
                 rotation_step=np.deg2rad(360),
                 viz=False):
        self.pose = pose
        self.scale = scale
        self.xyz_step = xyz_step
        self.hemisphere_angle = hemisphere_angle
        self.view_step = view_step
        self.rotation_step = rotation_step
        self.viz = viz

    def generate(self, viz=False) -> Tuple[np.ndarray, np.ndarray]:
        pose_matrix = get_matrix(self.pose)
        positions = table_point_generator(pose_matrix, self.scale, self.xyz_step)
        z_axis = pose_matrix[:3, 2]
        R_list = view_generator(
            inital_view=-1 * z_axis,
            hemisphere_angle=self.hemisphere_angle,
            view_step=self.view_step,
            rotation_step=self.rotation_step
        )
        orientations = np.array([matrix_to_rpy(R) for R in R_list])
        return positions, orientations

    @property
    def name(self) -> str:
        # Create a concise signature with key parameters
        xyz_str = f"{self.xyz_step:.2f}"
        scale_str = "x".join([f"{s:.1f}" for s in self.scale])
        angles_str = f"{np.rad2deg(self.hemisphere_angle):.0f}x{np.rad2deg(self.view_step):.0f}x{np.rad2deg(self.rotation_step):.0f}"
        return f"table_{scale_str}_{xyz_str}_{angles_str}"


if __name__ == "__main__":
    generator = TablePoseGenerator()
    positions, orientations = generator.generate()

    print(positions.shape)
    print(orientations.shape)
    print(generator.name)
