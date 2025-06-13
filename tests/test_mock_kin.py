#!/usr/bin/env python3

from bam_reachability.kinematics import MockKin  # Fill in with your actual functions
from bam_reachability.analysis.run_reachability_test import run_reachability_test

import pytest
from typing import Callable


OLD_MOCK_MAP_PATH = "/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/mock/mock_map_13_jun_2025.pkl"

def test_mock_kin_same():
    K = MockKin(L1=0.3, L2=0.3) # this needs to be the same as the orginal map or it will fail!

    run_reachability_test(
        name="MockKin Expecting Success",
        IK=K.IK,
        FK=K.FK,
        IK_ALT=K.IK,  # Replace with alternate IK if available
        FK_ALT=K.FK,
        map_old_path=OLD_MOCK_MAP_PATH,
        seed_reset_fn=K.seed_reset,
        reduce_count=None,
        visualize=False,
        expect_fail_consistency=False,
        expect_fail_deterministic=False,
        expect_fail_stability=False,
        expect_fail_compatibility=False,
    )

def test_mock_diff_l():
    K = MockKin(L1=0.3, L2=0.3) # this needs to be the same as the orginal map or it will fail!
    K_DIFF_L = MockKin(L1=0.5, L2=0.5) # this needs to be the same as the orginal map or it will fail!

    run_reachability_test(
        name="MockKin Different Link Lengths",
        IK=K.IK,
        FK=K.FK,
        IK_ALT=K_DIFF_L.IK,  # Replace with alternate IK if available
        FK_ALT=K_DIFF_L.FK,
        map_old_path=OLD_MOCK_MAP_PATH,
        seed_reset_fn=K.seed_reset,
        reduce_count=None,
        visualize=False,
        expect_fail_consistency=False,
        expect_fail_deterministic=False,
        expect_fail_stability=False,
        expect_fail_compatibility=True,
    )

def test_mock_no_reset():
    K = MockKin(L1=0.3, L2=0.3) # this needs to be the same as the orginal map or it will fail!

    run_reachability_test(
        name="MockKin No Reset",
        IK=K.IK,
        FK=K.FK,
        IK_ALT=K.IK,  # Replace with alternate IK if available
        FK_ALT=K.FK,
        map_old_path=OLD_MOCK_MAP_PATH,
        seed_reset_fn=None,
        reduce_count=None,
        visualize=False,
        expect_fail_consistency=False, # This will still be consitent b/c the randomness is in the IK
        expect_fail_deterministic=True, # Except to fail beacuse on the second run, the IK will return succesful orientations randomly
        expect_fail_stability=False, # This will succeed beacuse your using the same seed, and on the first run it will return the same result!
        expect_fail_compatibility=True, # Will fail beacuse on the third run it will return different random numbers ^ , its like its a different robot
    )

def test_mock_diff_seed():

    K_DIFF_SEED = MockKin(L1=0.3, L2=0.3, seed=1) 

    run_reachability_test(
        name="MockKin Different Seed",
        IK=K_DIFF_SEED.IK,
        FK=K_DIFF_SEED.FK,
        IK_ALT=K_DIFF_SEED.IK,  # Replace with alternate IK if available
        FK_ALT=K_DIFF_SEED.FK,
        map_old_path=OLD_MOCK_MAP_PATH,
        seed_reset_fn=K_DIFF_SEED.seed_reset,
        reduce_count=None,
        visualize=False,
        expect_fail_consistency=False, # This will still be consitent b/c the randomness is in the IK
        expect_fail_deterministic=False, # Will suceed because seed is reset between run 1 and 2
        expect_fail_stability=True, # Will fail beacuse different seed is used from old map and new run 1
        expect_fail_compatibility=False, # This will suceed beacuse IK_AL and IK are using the same seed
    )


if __name__ == "__main__":
    test_mock_kin_same()
    test_mock_diff_l()
    test_mock_no_reset()
    test_mock_diff_seed()


# colcon test --packages-select bam_reachability --pytest-args -m fast
# colcon test-result --all
# colcon test-result --all --verbose

# colcon test --packages-select bam_reachability --pytest-args="-m fast"
# colcon test --packages-select bam_reachability --pytest-args="-m slow"
