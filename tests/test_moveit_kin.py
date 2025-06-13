#!/usr/bin/env python3


import numpy as np
from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.visualizer import AlignedSlicer, colorize_map # Optional visualization
from bam_reachability.compare_maps import compare_map 
from bam_reachability.kinematics import MockKin  # Fill in with your actual functions

def test_moveit_kin():

    kinematics = MockKin(L1=0.3, L2=0.3) # this needs to be the same as the orginal map or it will fail!
    IK = kinematics.IK
    FK = kinematics.FK
    IK_PIN = IK # not implemented right now
    FK_PIN = FK 

    n_test_frames = 100
    map_old_path = "/home/bam/python_ws/bam_reachability/bam_reachability/robots/mock/mock_map_12_jun_2025.pkl"
    viz = False

    # Load previous analysis
    print("[Step] Loading old map...")
    map_old = ReachabilityMap.load(map_old_path, IK=IK, FK=FK)

    if viz:
        print("[Step] Visualizing old map...")
        points, colors = colorize_map(map_old, histogram=True)
        slicer = AlignedSlicer(points, step=0.05, colors=colors)
        slicer.run()

    # This screws up the order of the random seed! so cannot use for MockKin
    # map_old.reduce_random(n_test_frames) # to speed up testing

    if viz:
        print("[Step] Visualizing reduced map...")
        points, colors = colorize_map(map_old, histogram=True)
        slicer = AlignedSlicer(points, step=0.05, colors=colors)
        slicer.run()

    print("[Step] Recomputing reachability map 1...")
    map_1 = ReachabilityMap(frames=map_old.frames, orientations=map_old.orientations, IK=IK, FK=FK)
    kinematics.seed_reset() 

    print("[Step] Recomputing reachability map 2...")
    map_2 = ReachabilityMap(frames=map_old.frames, orientations=map_old.orientations, IK=IK, FK=FK)
    kinematics.seed_reset() 

    print("[Step] Computing reachability map using pinocchio IK/FK...")
    map_pin = ReachabilityMap(frames=map_old.frames, orientations=map_old.orientations, IK=IK_PIN, FK=FK_PIN)
    kinematics.seed_reset() 

    print("\n[TEST 1] Consistency (FK consistent w/ IK):")
    fk_consistent = np.mean(np.array([info["consistent"] for info in map_1.fk_map]))
    print(f"Mean FK consistency = {fk_consistent:.3f}")

    print("\n[TEST 2] Deterministic (same result twice):")
    deterministic_results = compare_map(map_1, map_2)
    print(deterministic_results)
    for key, value in deterministic_results.items():
        assert value == 1.0

    print("\n[TEST 3] Stable (same result as old):")
    stable_results = compare_map(map_1, map_old)
    print(stable_results)
    for key, value in stable_results.items():
        assert value == 1.0

    print("\n[TEST 4] Compatible (same result with pin IK/FK):")
    compatible_results = compare_map(map_1, map_pin)
    print(compatible_results)
    for key, value in compatible_results.items():
        assert value == 1.0

    print("\n[Done] All tests completed.")

if __name__ == "__main__":
    test_moveit_kin()
