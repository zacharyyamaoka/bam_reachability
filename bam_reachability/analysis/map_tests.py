#!/usr/bin/env python3

"""
Load an older analysis (map_old):
    - option for small or large
    - option to display the analysis before starting to test using viz

Then:
    - run the analysis agian on the same frames, twice -> (map_1, map_2)
    - run the analysis agian on the same frames using pin -> (map_pin)

Then check:

    1. Consistency:
    Mean map_1.fk_map["consistent"], should be 1

    2. Deterministic:
    Running the same analysis twice returns the same result
    compare(map_1, map_2)

    3. Stable:
    New map is same as old map
    compare(map_1, map_old)

    4. Compatible:
    Maps from different kinematics are the same
    compare(map_1, map_pin)

Careful!!!!! This will fail with the mock unless you seed it properly!!
"""

import numpy as np
from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.visualizer import O3DMapViewer, colorize_map # Optional visualization
from bam_reachability.analysis.compare_maps import compare_maps 
from typing import Callable, Optional
from bam_reachability.kin_wrapper import KinWrapper

def visualize_map(map: ReachabilityMap, visualise_step: float = 0.1):
    points, colors = colorize_map(map, histogram=True)
    O3DMapViewer(points, step=visualise_step, colors=colors).run()
        
def check_stable(map_old: ReachabilityMap, K: KinWrapper) -> tuple[bool, dict]:
    """ New map should be the same as the old map """
    map_new = ReachabilityMap(map_old.positions, map_old.orientations, IK=K.IK, FK=K.FK)
    map_new.calculate_ik_fk_map()
    return compare_maps(map_new, map_old)

def check_deterministic(map_ref: ReachabilityMap, K: KinWrapper) -> tuple[bool, dict]:
    """ Running Kinematics twice should return the same result """
    map_1 = ReachabilityMap(map_ref.positions, map_ref.orientations, IK=K.IK, FK=K.FK)
    map_1.calculate_ik_fk_map()
    map_2 = ReachabilityMap(map_ref.positions, map_ref.orientations, IK=K.IK, FK=K.FK)
    map_2.calculate_ik_fk_map()
    return compare_maps(map_1, map_2)

def check_consistent(map: ReachabilityMap) -> bool:
    """ ik_sol = IK(FK(ik_sol)), round trip conversion should have no effect"""
    fk_consistent = np.mean(np.array([info.consistent for info in map.fk_map]))
    return fk_consistent == 1.0

def check_compatible(map_ref: ReachabilityMap, K1: KinWrapper, K2: KinWrapper) -> tuple[bool, dict]:
    """ Maps from different kinematics (ie. moveit vs pin) should be the same """
    map_1 = ReachabilityMap(map_ref.positions, map_ref.orientations, IK=K1.IK, FK=K1.FK)
    map_1.calculate_ik_fk_map()
    map_2 = ReachabilityMap(map_ref.positions, map_ref.orientations, IK=K2.IK, FK=K2.FK)
    map_2.calculate_ik_fk_map()
    return compare_maps(map_1, map_2)


if __name__ == "__main__":
    from bam_reachability.kin_wrapper import MockKinWrapper  # Fill in with your actual functions

    K = MockKinWrapper(L1=0.3, L2=0.3) # this needs to be the same as the orginal map or it will fail!
    
    map_path = "/home/bam/python_ws/bam_reachability/maps/mock/Mock_table_0.8x0.4x0.1_0.25_0x0x360_map_13_jul_2025.pkl"
    map = ReachabilityMap.load(map_path, reduce_count=20)

    visualize_map(map)
