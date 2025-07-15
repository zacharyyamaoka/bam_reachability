#!/usr/bin/env python3

import numpy as np
from typing import Callable, Optional
import pickle
import datetime  # Add this at the top if not already
import os  

from bam_reachability.utils.math_utils import pose_is_close, xyz_R_to_matrix

"""
Make sure that orientation positions and IK tip are aligned

- generally IK tip is has +Z pointing out of the final link


#TODO make it so each frame could potetially have a different number of orientations...
"""

class IKInfo:
    def __init__(self, n: int):
        self.success = [0] * n # by default it's failed
        self.ik_sols = [None] * n

    def update(self, orient_idx, ik_success, ik_sol):
        if ik_success:
            self.success[orient_idx] = 1
            self.ik_sols[orient_idx] = ik_sol
        else:
            pass #by default ik_sols is None and success is 0

    def to_numpy(self):
        self.success = np.array(self.success)
        return self

class FKInfo:
    def __init__(self, n: int):
        self.success = [0] * n # by default it's failed
        self.consistent = [1] * n # by default it's consistent
        self.fk_sols = [None] * n

    def update(self, orient_idx, fk_success, consistent, fk_sol):

        self.success[orient_idx] = fk_success
        self.consistent[orient_idx] = consistent
        self.fk_sols[orient_idx] = fk_sol

    # Turn into numpy arrays so can easily np.sum() to calculate 
    def to_numpy(self):
        self.success = np.array(self.success)
        self.consistent = np.array(self.consistent)
        return self

class ReachabilityMap():

    def __init__(self,
                 positions: np.ndarray,
                 orientations: np.ndarray | list[np.ndarray],
                 IK, # function
                 FK, # function
                 ):

        assert positions.ndim == 2 and positions.shape[1] == 3  # shape: (N, 3)
        self.positions = positions 
        self.n_positions = self.positions.shape[0]

        # Orientations are either shared accross all positions (N, 3, 3) or unique per position List[(N1, 3, 3), (N2, 3, 3), ...]
        if isinstance(orientations, list):
            raise NotImplementedError("List of orientations is not implemented yet")
            self.same_orientations = False

        assert orientations.ndim == 3 and orientations.shape[1] == 3 and orientations.shape[2] == 3, "Orientations must be of shape (N, 3, 3)"
        self.same_orientations = True
        self.n_orientations = orientations.shape[0]
        self.orientations = orientations


        self.total_count = self.n_positions * self.n_orientations

        self.ik_map: list[IKInfo] = []
        self.fk_map: list[FKInfo] = []

        self.IK = IK
        self.FK = FK

    def get_pose_matrix(self, pos_idx: int, orient_idx: int) -> np.ndarray:
        position = self.positions[pos_idx,:] # shape (3,)
        orientation = self.orientations[orient_idx, :, :]
        return xyz_R_to_matrix(position, orientation)
    
    def calculate_ik_fk_map(self, verbose: bool = False) -> None:

        self.ik_map = []
        self.fk_map = []
        count = 0

        for i in range(self.n_positions): 
            ik_info = IKInfo(self.n_orientations)
            fk_info = FKInfo(self.n_orientations)

            for j in range(self.n_orientations):
                count += 1

                pose_matrix = self.get_pose_matrix(i, j)

                ik_success, ik_sol = self.IK(pose_matrix)
                ik_info.update(j, ik_success, ik_sol)

                fk_success, consistent, fk_sol = self.check_fk_consistency(pose_matrix, ik_success, ik_sol)
                fk_info.update(j, fk_success, consistent, fk_sol)
  
                if verbose and (count % 1000 == 0 or count == self.total_count):
                    print(f"Processed {count} / {self.total_count} poses")

            self.ik_map.append(ik_info.to_numpy())
            self.fk_map.append(fk_info.to_numpy())

        assert len(self.ik_map) == self.n_positions

    def check_fk_consistency(self, pose_matrix, ik_success, ik_sol) -> tuple[int, int, np.ndarray | None]: # return fk_sol, consitent, fk_success
        fk_sol = None
        consistent = 1
        fk_success = 0

        if not ik_success: # the ik_sol is ill-defined here so no point in checking!
            return fk_success, consistent, fk_sol
                
        fk_success, fk_sol = self.FK(ik_sol)

        if not fk_success:
            consistent = 0
            return fk_success, consistent, None
      
        if not np.allclose(pose_matrix, fk_sol):
            consistent = 0

        return fk_success, consistent, fk_sol
    
    def calculate_fk_map(self):
        """ Not needed as done at same time as IK"""

        self.fk_map = []
        count = 0

        for i in range(self.n_positions): 

            ik_info = self.ik_map[i]
            fk_info = FKInfo(self.n_orientations)

            for j in range(self.n_orientations):
                count += 1

                ik_success = ik_info.success[j]
                if ik_success:

                    pose_matrix = self.get_pose_matrix(i, j)

                    ik_sol = ik_info.ik_sols[j] # stored solution for this pose

                    fk_sol, consistent, fk_success = self.check_fk_consistency(pose_matrix, ik_success, ik_sol)
                    fk_info.update(j, fk_success, consistent, fk_sol)

            self.fk_map.append(fk_info.to_numpy())
    
    def save(self, file_path: str, timestamp=True):

        if timestamp:
            date_str = datetime.datetime.now().strftime("%d_%b_%Y").lower()  # e.g., '03_feb_2025'
            if file_path.endswith(".pkl"): 
                file_path = file_path[:-4] + f"_{date_str}.pkl"
            else:
                file_path = file_path + f"_{date_str}.pkl"

        dir_path = os.path.dirname(file_path)
        if not os.path.exists(dir_path):
            raise FileNotFoundError(f"[Error] Save directory does not exist: {dir_path}")

        data = {
            "positions": self.positions,
            "orientations": self.orientations,
            "same_orientations": self.same_orientations,  # <-- NEW
            "ik_map": self.ik_map,
            "fk_map": self.fk_map,
        }
        with open(file_path, "wb") as f:
            pickle.dump(data, f)
        print(f"[Saved] Reachability map saved to {file_path}")

    @staticmethod
    def load(file_path: str, reduce_count: Optional[int] = None):
        with open(file_path, "rb") as f:
            data = pickle.load(f)

        map = ReachabilityMap(data["positions"], data["orientations"], None, None)

        map.same_orientations = data.get("same_orientations", False)  # <-- NEW (fallback to False)
        map.ik_map = data["ik_map"]
        map.fk_map = data["fk_map"]


        if reduce_count is not None:
            map.reduce_random(reduce_count)

        print(f"[Loaded] Reachability map loaded from {file_path}")
        return map
        
    def reduce_random(self, num_positions: int, seed: Optional[int] = None):
        """
        Randomly sample a subset of positions to reduce the reachability map.

        Args:
            num_positions (int): Number of positions to keep.
            seed (int, optional): Random seed for reproducibility.
        """
        if num_positions >= self.n_positions:
            print(f"[Skip] Requested {num_positions} positions, but map has only {self.n_positions}. No reduction done.")
            return

        if seed is not None:
            np.random.seed(seed)

        selected_indices = np.random.choice(self.n_positions, size=num_positions, replace=False)
        selected_indices.sort()  # Optional: keep original order

        self.reduce(selected_indices)

    def reduce(self, frame_indices: np.ndarray):
        """
        Reduce the reachability map to only include positions at the given indices.

        Args:
            frame_indices (np.ndarray): Array of indices to keep (subset of original frame indices)
        """
        print(f"[Reduced] Reducing map from {self.n_positions} to {len(frame_indices)} positions...")
        self.positions = self.positions[frame_indices]
        self.ik_map = [self.ik_map[i] for i in frame_indices]
        self.fk_map = [self.fk_map[i] for i in frame_indices]

        if not self.same_orientations:
            raise NotImplementedError("Reducing map with different orientations is not implemented yet")
            self.orientations = self.orientations[frame_indices]

        self.n_positions = self.positions.shape[0]
        self.n_orientations = self.orientations.shape[0] 
        self.total_count = self.n_positions * self.n_orientations

    def get_position(self, frame_index: int) -> np.ndarray:
        return self.positions[frame_index]

    def get_orientation(self, frame_index: int, orient_index: int) -> np.ndarray:
        if self.same_orientations:
            return self.orientations[frame_index, orient_index]
        else:
            return self.orientations[orient_index]

    def get_sol_or_none(self, sol_list) -> np.ndarray | None:
        for sol in sol_list:
            if sol is not None:
                return sol
        return None

    def sample_valid_fk_sol(self) -> np.ndarray:
        while True:
            frame_index = np.random.randint(0, self.n_positions)
            fk_info = self.fk_map[frame_index]
            for orient_index, fk_sol in enumerate(fk_info.fk_sols):
                if fk_sol is not None:
                    return fk_sol
            # If no valid fk_sol found, try another random frame_index

    def sample_valid_ik_sol(self) -> np.ndarray:
        while True:
            frame_index = np.random.randint(0, self.n_positions)
            ik_info = self.ik_map[frame_index]
            for orient_index, ik_sol in enumerate(ik_info.ik_sols):
                if ik_sol is not None:
                    return ik_sol
            # If no valid fk_sol found, try another random frame_index

    def mask_success_mean(self, threshold: float = 1.0):

        scores = np.array([np.mean(ik_info["success"]) for ik_info in self.ik_map])  # (N,)
        mask = scores >= threshold
        frame_indices = np.nonzero(mask)[0]

        if len(frame_indices) == 0:
            print(f"[Filtered] No positions met the threshold of {threshold}. Skipping as would reduce map to 0")
            return

        self.reduce(frame_indices)


def make_map_path(curr__file__, folder_name, kinematic_name, generator_name) -> str:
    
    current = os.path.abspath(curr__file__)
    parent1 = os.path.dirname(current)
    parent2 = os.path.dirname(parent1)
    parent3 = os.path.dirname(parent2)

    base_dir = parent2
    file_path = os.path.join(base_dir, 'maps', folder_name, f'{kinematic_name}_{generator_name}_map')
    print("Created file path: ", file_path)

    return file_path