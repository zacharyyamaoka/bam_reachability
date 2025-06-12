#!/usr/bin/env python3

import numpy as np
from typing import Callable, List, Dict, Tuple

# --- Utility functions ---

def normalize_quat(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)

def angle_between_quats(q1: np.ndarray, q2: np.ndarray) -> float:
    """Returns angular distance between two quaternions in radians."""
    q1 = normalize_quat(q1)
    q2 = normalize_quat(q2)
    return 2 * np.arccos(np.clip(np.abs(np.dot(q1, q2)), -1.0, 1.0))

def poses_equal(p1: Dict, p2: Dict, pos_tol=1e-4, ang_tol=1e-3) -> bool:
    pos_diff = np.linalg.norm(p1["position"] - p2["position"])
    ang_diff = angle_between_quats(p1["orientation"], p2["orientation"])
    return pos_diff < pos_tol and ang_diff < ang_tol

def joints_equal(q1: np.ndarray, q2: np.ndarray, tol=1e-4) -> bool:
    return np.allclose(q1, q2, atol=tol)


# --- Pose/Joint Map Abstraction ---

class IKFKMap:
    def __init__(self, ik_fn: Callable[[Dict], Tuple[np.ndarray, bool]],
                       fk_fn: Callable[[np.ndarray], Dict],
                       name="unknown"):
        self.ik = ik_fn
        self.fk = fk_fn
        self.name = name


# --- Test Runner Functions ---

def test_round_trip(pose: Dict, map_: IKFKMap) -> bool:
    q_ik, success = map_.ik(pose)
    if not success:
        return False
    pose_fk = map_.fk(q_ik)
    return poses_equal(pose, pose_fk)

def test_repeatability(pose: Dict, map_: IKFKMap) -> bool:
    q1, s1 = map_.ik(pose)
    q2, s2 = map_.ik(pose)
    return s1 and s2 and joints_equal(q1, q2)

def test_temporal_consistency(pose_old: Dict, q_old: np.ndarray, map_: IKFKMap) -> bool:
    q_new, success = map_.ik(pose_old)
    pose_fk = map_.fk(q_old)
    return success and joints_equal(q_old, q_new) and poses_equal(pose_old, pose_fk)

def test_cross_map(pose: Dict, map_a: IKFKMap, map_b: IKFKMap) -> bool:
    q_a, s_a = map_a.ik(pose)
    q_b, s_b = map_b.ik(pose)
    if not (s_a and s_b):
        return False
    pose_fk_a = map_a.fk(q_a)
    pose_fk_b = map_b.fk(q_b)
    return joints_equal(q_a, q_b) and poses_equal(pose_fk_a, pose_fk_b)


# --- Example Runner ---

def run_tests_on_dataset(poses: List[Dict], maps: List[IKFKMap], old_pose_data=None):
    for pose in poses:
        for map_ in maps:
            print(f"\n[{map_.name}] Round-trip: ", test_round_trip(pose, map_))
            print(f"[{map_.name}] Repeatability: ", test_repeatability(pose, map_))

        if old_pose_data:
            pose_old, q_old = old_pose_data
            for map_ in maps:
                print(f"[{map_.name}] Temporal Consistency: ", test_temporal_consistency(pose_old, q_old, map_))

        if len(maps) > 1:
            print(f"Cross Map ({maps[0].name} vs {maps[1].name}): ", test_cross_map(pose, maps[0], maps[1]))
