#!/usr/bin/env python3

from bam_reachability.kin_wrapper import OffsetWristKinWrapper  # Fill in with your actual functions
from bam_reachability.analysis.run_reachability_test import run_reachability_test

OLD_MAP_PATH = "/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur_offset_wrist_map_13_jun_2025.pkl"


def test_offset_wrist_kin():
    K = OffsetWristKinWrapper(arm="ur")
    run_reachability_test(
        name="Offset WRist",
        IK=K.IK,
        FK=K.FK,
        IK_ALT=K.IK,  # Replace with alternate IK if available
        FK_ALT=K.FK,
        map_old_path=OLD_MAP_PATH,
        seed_reset_fn=None,
        reduce_count=None,
        visualize=False,
        expect_fail_consistency=False,
        expect_fail_deterministic=False,
        expect_fail_stability=False,
        expect_fail_compatibility=False,
    )


if __name__ == "__main__":
    test_offset_wrist_kin()

