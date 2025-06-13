#!/usr/bin/env python3

import numpy as np
from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.utils import pose_is_close, ik_sol_is_close, fk_sol_is_close

def new_diff_info(n_orientations) -> dict:
    diff_info = {
        "pose_success":[0] * n_orientations, 
        "ik_success":[0] * n_orientations, 
        "ik_sol_success":[0] * n_orientations, 
        "fk_success":[0] * n_orientations, 
        "fk_sol_success":[0] * n_orientations, 
        }
    return diff_info

def compare_map(map_1: ReachabilityMap, map_2: ReachabilityMap):
    """
    Compares two ReachabilityMaps and returns mean values for each consistency metric.
    All returned values should be 1.0 if maps are identical.
    """

    assert map_1.n_frames == map_2.n_frames
    assert map_1.n_orientations == map_2.n_orientations

    diff_map = []

    frames_1, orientations_1 = map_1.frames, map_1.orientations
    ik_map_1, fk_map_1 = map_1.ik_map, map_1.fk_map

    frames_2, orientations_2 = map_2.frames, map_2.orientations
    ik_map_2, fk_map_2 = map_2.ik_map, map_2.fk_map

    # First check that lengths are all equal
    # Not needed actually, as long as its correct doesn't matter if its longer
    count = 0
    for i in range(map_1.n_frames): 
        frame_1 = frames_1[i,:] # shape (3,)
        frame_2 = frames_2[i,:] # shape (3,)

        ik_info_1 = ik_map_1[i]
        ik_info_2 = ik_map_2[i]

        fk_info_1 = fk_map_1[i]
        fk_info_2 = fk_map_2[i]

        diff_info = new_diff_info(map_1.n_orientations)

        for j in range(map_1.n_orientations):
            count += 1

            # Check frame poses are the same. 
            # Important that inputs are the same, or outputs are not likely to be!
            orientation_1 = orientations_1[j,:] # shape (3,)
            orientation_2 = orientations_2[j,:] # shape (3,)
            pose_1 = np.hstack((frame_1, orientation_1)) # shape (6,)
            pose_2 = np.hstack((frame_2, orientation_2)) # shape (6,)
            diff_info["pose_success"][j] = pose_is_close(pose_1, pose_2)

            # Check IK results are the Same
            ik_success_1 = ik_info_1["success"][j] 
            ik_success_2 = ik_info_2["success"][j] 
            ik_success_same = (ik_success_1 == ik_success_2)
            if not ik_success_same:
                print("[ERROR] IK success not the same")
                print(f"ik_success_1: {ik_success_1} {ik_info_1["ik_sols"][j]}")
                print(f"ik_success_1: {ik_success_2} {ik_info_2["ik_sols"][j]}")

            diff_info["ik_success"][j] = ik_success_same

            ik_sol_1 = ik_info_1["ik_sols"][j] 
            ik_sol_2 = ik_info_2["ik_sols"][j] 
            diff_info["ik_sol_success"][j] = ik_sol_is_close(ik_sol_1, ik_sol_2)

            # Check FK results are the Same
            # This may fail if they don't have the same collision checking anymore!
            fk_success_1 = fk_info_1["success"][j] 
            fk_success_2 = fk_info_2["success"][j] 
            diff_info["fk_success"][j] = (fk_success_1 == fk_success_2)

            fk_sol_1 = fk_info_1["fk_sols"][j] 
            fk_sol_2 = fk_info_2["fk_sols"][j] 
            diff_info["fk_sol_success"][j] = fk_sol_is_close(fk_sol_1, fk_sol_2)

        diff_info["ik_success"] = np.array(diff_info["ik_success"])
        diff_info["ik_sol_success"] = np.array(diff_info["ik_sol_success"])
        diff_info["fk_success"] = np.array(diff_info["fk_success"])
        diff_info["fk_sol_success"] = np.array(diff_info["fk_sol_success"])

        diff_map.append(diff_info)

    # Initialize empty lists to collect values
    summary = {}
    for key in diff_map[0].keys():
        summary[key] = []

    # Collect values from each frame
    for i in range(map_1.n_frames): 
        for key in diff_map[0].keys():
            summary[key].append(diff_map[i][key])

    # Compute mean across all frames and orientations
    for key in summary.keys():
        summary[key] = np.mean(np.array(summary[key]))

    return summary

