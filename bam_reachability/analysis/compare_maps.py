#!/usr/bin/env python3

import numpy as np
from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.utils.math_utils import pose_is_close, ik_sol_is_close, fk_sol_is_close, pose_matrix_is_close

class DiffInfo:
    def __init__(self, n_orientations):
        self.pose_success = [0] * n_orientations
        self.ik_success = [0] * n_orientations
        self.ik_sol_success = [0] * n_orientations
        self.fk_success = [0] * n_orientations
        self.fk_sol_success = [0] * n_orientations
        self.fk_consistent = [0] * n_orientations

    def to_numpy(self):
        self.pose_success = np.array(self.pose_success)
        self.ik_success = np.array(self.ik_success)
        self.ik_sol_success = np.array(self.ik_sol_success)
        self.fk_success = np.array(self.fk_success)
        self.fk_sol_success = np.array(self.fk_sol_success)
        self.fk_consistent = np.array(self.fk_consistent)
        return self

    def get_keys(self) -> list[str]:
        return [
            "pose_success",
            "ik_success",
            "ik_sol_success",
            "fk_success",
            "fk_sol_success",
            "fk_consistent",
        ]

    def to_summary_array(self) -> np.ndarray:
        arr = np.array([
            np.mean(self.pose_success),
            np.mean(self.ik_success),
            np.mean(self.ik_sol_success),
            np.mean(self.fk_success),
            np.mean(self.fk_sol_success),
            np.mean(self.fk_consistent),
        ])
        return arr

def compare_maps(map_1: ReachabilityMap, map_2: ReachabilityMap, verbose=False) -> tuple[bool, dict]:
    """
    Compares two ReachabilityMaps and returns mean values for each consistency metric.
    All returned values should be 1.0 if maps are identical.
    """
    # If these do not match, then you cannot compare below
    assert map_1.n_positions == map_2.n_positions
    assert map_1.n_orientations == map_2.n_orientations
    assert map_1.same_orientations == map_2.same_orientations

    diff_map: list[DiffInfo] = []

    # First check that lengths are all equal
    # Not needed actually, as long as its correct doesn't matter if its longer
    count = 0
    for i in range(map_1.n_positions): 

        ik_info_1 = map_1.ik_map[i]
        fk_info_1 = map_1.fk_map[i]

        ik_info_2 = map_2.ik_map[i]
        fk_info_2 = map_2.fk_map[i]

        diff_info = DiffInfo(map_1.n_orientations)

        for j in range(map_1.n_orientations):
            count += 1

            pose_1 = map_1.get_pose_matrix(i, j)
            pose_2 = map_2.get_pose_matrix(i, j)

            diff_info.pose_success[j] = pose_matrix_is_close(pose_1, pose_2, verbose)
            diff_info.ik_success[j] = (ik_info_1.success[j] == ik_info_2.success[j])
            diff_info.ik_sol_success[j] = ik_sol_is_close(ik_info_1.ik_sols[j], ik_info_2.ik_sols[j], verbose)

            # You may think FK always succesful, but could fail or be different if collision checking has changed 
            diff_info.fk_success[j] = (fk_info_1.success[j] == fk_info_2.success[j])
            diff_info.fk_consistent[j] = (fk_info_1.consistent[j] == fk_info_2.consistent[j])
            diff_info.fk_sol_success[j] = fk_sol_is_close(fk_info_1.fk_sols[j], fk_info_2.fk_sols[j], verbose)

    
        diff_map.append(diff_info.to_numpy())

    summary_array = np.zeros((map_1.n_positions, 6))
    for i in range(map_1.n_positions):
        summary_array[i] = diff_map[i].to_summary_array()
    summary_array = np.mean(summary_array, axis=0)

    summary_keys = diff_map[0].get_keys()
    summary_dict = dict(zip(summary_keys, summary_array))
    if verbose:
        print(summary_dict)

    return np.mean(summary_array) == 1.0, summary_dict

