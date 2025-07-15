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
from bam_reachability.visualizer import Open3DMapViewer, colorize_map # Optional visualization
from bam_reachability.analysis.compare_maps import compare_maps 
from typing import Callable, Optional


def run_reachability_test(
    name: str,
    map_old_path: str,
    IK: Callable,
    FK: Callable,
    IK_ALT: Callable,
    FK_ALT: Callable,
    seed_reset_fn: Optional[Callable] = None,
    reduce_count: Optional[int] = None,
    visualize: bool = False,
    visualise_step: float = 0.1,
    expect_fail_consistency=False,
    expect_fail_deterministic=False,
    expect_fail_stability=False,
    expect_fail_compatibility=False,
    skip_keys=[]
):
    print(f"[TESTING] Starting tests for '{name}'")

    print("[Step] Loading old map...")
    map_old = ReachabilityMap.load(map_old_path, IK=IK, FK=FK)

    if visualize:
        print("[Step] Visualizing old map...")
        points, colors = colorize_map(map_old, histogram=True)
        Open3DMapViewer(points, step=visualise_step, colors=colors).run()

    if reduce_count is not None:
        print(f"[Step] Reducing map to {reduce_count} test frames...")
        map_old.reduce_random(reduce_count)

    if visualize and reduce_count is not None:
        print("[Step] Visualizing reduced map...")
        points, colors = colorize_map(map_old, histogram=True)
        Open3DMapViewer(points, step=visualise_step, colors=colors).run()

    print("[Step] Recomputing reachability map 1...")
    if seed_reset_fn: seed_reset_fn()
    map_1 = ReachabilityMap(map_old.frames, map_old.orientations, IK=IK, FK=FK)

    print("[Step] Recomputing reachability map 2...")
    if seed_reset_fn: seed_reset_fn()
    map_2 = ReachabilityMap(map_old.frames, map_old.orientations, IK=IK, FK=FK)

    print("[Step] Recomputing reachability map with alternate IK/FK...")
    if seed_reset_fn: seed_reset_fn()
    map_alt = ReachabilityMap(map_old.frames, map_old.orientations, IK=IK_ALT, FK=FK_ALT)

    def check(name, value, expected=1.0, allow_fail=False):
        if skip_keys:
            if name in skip_keys:
                # expected = 1.0
                # allow_fail = False
                print(f"[SKIP KEY] {name}")
                return

        if value == expected:
            if allow_fail:
                raise AssertionError(f"[UNEXPECTED PASS] {name}: Got {value}, but was expected to fail.")
            print(f"[PASS] {name}")
        elif allow_fail:
            print(f"[EXPECTED FAIL] {name}: Got {value}, expected {expected}")
        else:
            raise AssertionError(f"[FAILED] {name}: Got {value}, expected {expected}")

    print("\n[TEST 1] Consistency (FK consistent with IK):")
    fk_consistent = np.mean([info["consistent"] for info in map_1.fk_map])
    print(f"Mean FK consistency: {fk_consistent:.3f}")
    check("Consistency", fk_consistent, expected=1.0, allow_fail=expect_fail_consistency)

    print("\n[TEST 2] Deterministic (repeatable result):")
    det = compare_maps(map_1, map_2)
    print(det)
    for key, value in det.items():
        check(f"Deterministic[{key}]", value, expected=1.0, allow_fail=expect_fail_deterministic)

    print("\n[TEST 3] Stable (matches saved map):")
    stable = compare_maps(map_1, map_old)
    print(stable)
    for key, value in stable.items():
        check(f"Stable[{key}]", value, expected=1.0, allow_fail=expect_fail_stability)

    print("\n[TEST 4] Compatible (alternate kinematics produce same result):")
    compat = compare_maps(map_1, map_alt)
    print(compat)
    for key, value in compat.items():
        check(f"Compatible[{key}]", value, expected=1.0, allow_fail=expect_fail_compatibility)

    print(f"\n[RESULT] All tests completed for '{name}'.")


if __name__ == "__main__":
    from bam_reachability.kin_wrapper import MockKinWrapper  # Fill in with your actual functions

    K = MockKinWrapper(L1=0.3, L2=0.3) # this needs to be the same as the orginal map or it will fail!
    
    map_path = "/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/mock/mock_map_13_jun_2025.pkl"
    run_reachability_test(
        name="MockKin Expecting Success",
        IK=K.IK,
        FK=K.FK,
        IK_ALT=K.IK,  # Replace with alternate IK if available
        FK_ALT=K.FK,
        map_old_path=map_path,
        seed_reset_fn=K.seed_reset,
        reduce_count=None,
        visualize=False,
        visualise_step=0.1,
        expect_fail_consistency=False,
        expect_fail_deterministic=False,
        expect_fail_stability=False,
        expect_fail_compatibility=False,
    )
