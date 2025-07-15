#!/usr/bin/env python3

from bam_reachability.kin_wrapper import KinWrapper
from bam_reachability.generators import TablePoseGenerator, visualize_frames
from bam_reachability.utils.math_utils import get_matrix, xyz_R_to_matrix
import numpy as np
from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.visualization import colorize_reachability, colorize_inconsistency, Open3DMapViewer
from bam_reachability.generators import mask_R_list_by_angle
from bam_reachability.reachability_map import IKInfo
# Load Kinematics

# Create Generic Map

# Create Hemisphere Map

class PoseSpace():
    def __init__(self, positions: np.ndarray, orientations: np.ndarray):
        self.positions = positions
        self.orientations = orientations
        
        # Initialize shuffled indices for sampling without replacement
        self._reset_sampling_state()

        print("Creating pose space with {} positions and {} orientations".format(len(positions), len(orientations)))

    def _reset_sampling_state(self):
        """Reset and shuffle the sampling indices for positions only."""
        self.available_pos_indices = list(np.arange(len(self.positions)))
        np.random.shuffle(self.available_pos_indices)

    def reset_if_needed(self, n: int):
        n_pos_available = len(self.available_pos_indices)
        
        if n_pos_available < n:
            print(f"Warning: Requested {n} samples but only {n_pos_available} positions available. Resetting and continuing.")
            self._reset_sampling_state()

    def _sample(self, n: int) -> list[np.ndarray]:
        # Pop position indices from the shuffled array (no replacement)
        pos_indices = [self.available_pos_indices.pop() for _ in range(n)]
        
        # Sample orientations with replacement (can repeat)
        orient_indices = np.random.choice(len(self.orientations), n, replace=True)
        
        pos = self.positions[pos_indices]
        orient = self.orientations[orient_indices]

        poses = []

        for i in range(n):
            poses.append(xyz_R_to_matrix(pos[i], orient[i]))

        return poses

    def sample(self, n=1) -> np.ndarray | list[np.ndarray]:

        self.reset_if_needed(n)

        poses = self._sample(n)

        if n == 1:
            return poses[0]

        return poses

    def reset_sampling(self):
        """Reset the sampling state to allow all position samples to be used again."""
        self._reset_sampling_state()

    def remaining_samples(self) -> int:
        """Return the number of remaining position samples."""
        return len(self.available_pos_indices)

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


"""

    I Kinda feel that the reachability map should know the pose matrix...
    But its also powerful to be able to set it differently... you can have many different pose_matrixes actually for the same map.... which is cool...

    Doing reachable is complicated because each position orientation could be different... the other ones is much simpler... beacuse all the orientations need to be the same!

    The responsability of this class is to turn a map into workspace that can be sampled from!
"""
class WorkspaceInfo():
    def __init__(self, dexterous_score, hemisphere_score, four_dof_score):
        self.dexterous_score = float(dexterous_score)
        self.hemisphere_score = float(hemisphere_score)
        self.four_dof_score = float(four_dof_score)

# self.table_z_axis = table_pose_matrix[:3, 2]
# self.four_dof_view = -1 * self.table_z_axis

class ReachabilityWorkspace():
    def __init__(self, full_hemisphere_map: ReachabilityMap, four_dof_view: np.ndarray):
        """ z_axis of table_pose_matrix should be pointing up and out of table surface """

        self.map = full_hemisphere_map
        assert self.map.same_orientations

        self.four_dof_view = four_dof_view
        assert self.four_dof_view.shape == (3,) 

        # self.workspace_info: list[WorkspaceInfo] = []

        self.dexterous_pos_mask = []
        self.reachable_pos_mask = []
        self.hemisphere_pos_mask = []
        self.four_dof_pos_mask = []

        self.dexterous_pos_idx = []
        self.reachable_pos_idx = []
        self.hemisphere_pos_idx = []
        self.four_dof_pos_idx = []   

        self.dexterous_orient_mask = mask_R_list_by_angle(self.map.orientations, self.four_dof_view, np.deg2rad(180))
        self.hemisphere_orient_mask = mask_R_list_by_angle(self.map.orientations, self.four_dof_view, np.deg2rad(90))
        self.four_dof_orient_mask = mask_R_list_by_angle(self.map.orientations, self.four_dof_view, np.deg2rad(5))

        print(self.map.orientations.shape)
        print(self.map.orientations[self.four_dof_orient_mask].shape)
        print(self.map.orientations[self.hemisphere_orient_mask].shape)

        for i in np.arange(self.map.n_positions):
            ik_sol: IKInfo = self.map.ik_map[i]
            dex_score = np.mean(ik_sol.success)
            hemisphere_score = np.mean(ik_sol.success[self.hemisphere_orient_mask])
            four_dof_score = np.mean(ik_sol.success[self.four_dof_orient_mask])

            dex_success = dex_score==1
            reachable_success = dex_score>0
            hemisphere_success = hemisphere_score==1
            four_dof_success = four_dof_score==1

            self.dexterous_pos_mask.append(dex_success)
            self.reachable_pos_mask.append(reachable_success)
            self.hemisphere_pos_mask.append(hemisphere_success)
            self.four_dof_pos_mask.append(four_dof_success)

            if dex_success: self.dexterous_pos_idx.append(i)
            if reachable_success: self.reachable_pos_idx.append(i)
            if hemisphere_success: self.hemisphere_pos_idx.append(i)
            if four_dof_success: self.four_dof_pos_idx.append(i)

        self.dexterous_pos_mask = np.array(self.dexterous_pos_mask)
        self.reachable_pos_mask = np.array(self.reachable_pos_mask)
        self.hemisphere_pos_mask = np.array(self.hemisphere_pos_mask)
        self.four_dof_pos_mask = np.array(self.four_dof_pos_mask)

        n_dexterous = np.sum(self.dexterous_pos_mask)
        n_reachable = np.sum(self.reachable_pos_mask)
        n_hemisphere = np.sum(self.hemisphere_pos_mask)
        n_four_dof = np.sum(self.four_dof_pos_mask)

        print(f"n_dexterous: {n_dexterous}/{len(self.dexterous_pos_mask)}")
        print(f"n_reachable: {n_reachable}/{len(self.reachable_pos_mask)}")
        print(f"n_hemisphere: {n_hemisphere}/{len(self.hemisphere_pos_mask)}")
        print(f"n_four_dof: {n_four_dof}/{len(self.four_dof_pos_mask)}")

        assert n_reachable >= n_four_dof >= n_hemisphere >= n_dexterous

        self.dexterous = PoseSpace(self.map.positions[self.dexterous_pos_mask], self.map.orientations)
        self.hemisphere = PoseSpace(self.map.positions[self.hemisphere_pos_mask], self.map.orientations[self.hemisphere_orient_mask])
        self.four_dof = PoseSpace(self.map.positions[self.four_dof_pos_mask], self.map.orientations[self.four_dof_orient_mask])

        self.reachable_points = self.map.positions[self.reachable_pos_mask]

    
    def sample_q(self, workspace="dexterous"):

        pos_idx_list = getattr(self, f"{workspace}_pos_idx")
        pos_idx = np.random.choice(pos_idx_list)
        orient_mask = getattr(self, f"{workspace}_orient_mask")
        orient_idx_list = np.arange(len(orient_mask))[orient_mask]
        orient_idx = np.random.choice(orient_idx_list)

        return self.map.ik_map[pos_idx].ik_sols[orient_idx]

    def sample_dexterous_q(self):
        return self.sample_q("dexterous")

    def sample_hemisphere_q(self):
        return self.sample_q("hemisphere")

    def sample_four_dof_q(self):
        return self.sample_q("four_dof")
        
    def calculate(self):
        pass

    def save(self, file_path: str):
        pass

    def load(self, file_path: str):
        pass



if __name__ == "__main__":

    map = ReachabilityMap.load("/home/bam/python_ws/bam_reachability/maps/ur/ur5e/ur5e_table_1.2x1.2x1.0_0.10_180x45x360_15_jul_2025.pkl")

    table_pose_matrix = np.eye(4)
    z_axis = -1 * table_pose_matrix[:3, 2]

    workspace = ReachabilityWorkspace(map, z_axis)

    for i in range(10):
        print(workspace.dexterous.sample())


    for i in range(10):
        print(workspace.sample_q())