#!/usr/bin/env python3

import rclpy
from bam_reachability.kin_wrapper import MoveItKinWrapper, OffsetWristKinWrapper  # Fill in with your actual functions
from bam_reachability.analysis.run_reachability_test import run_reachability_test


def test_moveit_kin():
    map_path = "/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur_moveit_map_13_jun_2025.pkl"
    K_moveit = MoveItKinWrapper(arm="ur", ns="")
    K_offset_wrist = OffsetWristKinWrapper(arm="ur")

    run_reachability_test(
        name="Moveit Kinematics",
        IK=K_moveit.IK,
        FK=K_moveit.FK,
        IK_ALT=K_offset_wrist.IK,  # Replace with alternate IK if available
        FK_ALT=K_offset_wrist.FK,
        map_old_path=map_path,
        seed_reset_fn=None,
        reduce_count=None,
        visualize=False,
        expect_fail_consistency=False,
        expect_fail_deterministic=False,
        expect_fail_stability=False,
        expect_fail_compatibility=False,
    )


if __name__ == "__main__":


    rclpy.init(args=None)


    test_moveit_kin()

    rclpy.shutdown()
    exit(0)
