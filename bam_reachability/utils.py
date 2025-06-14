#!/usr/bin/env python3

import numpy as np

from transforms3d.euler import euler2quat

def quats_equal_up_to_sign(rpy1, rpy2, tol=1e-6):
    """
    Check if two RPY Euler angles result in equivalent rotations by comparing
    the resulting quaternions up to sign.
    
    Args:
        rpy1, rpy2: iterable of 3 floats (roll, pitch, yaw) in radians
        tol: comparison tolerance
    
    Returns:
        True if rotations are equivalent, False otherwise
    """
    q1 = np.array(euler2quat(*rpy1, axes='sxyz'))  # [w, x, y, z]
    q2 = np.array(euler2quat(*rpy2, axes='sxyz'))

    # Rearrange to [x, y, z, w] if needed
    # q1 = np.roll(q1, -1)
    # q2 = np.roll(q2, -1)

    return np.allclose(q1, q2, atol=tol) or np.allclose(q1, -q2, atol=tol)

def check_for_none(sol_1, sol_2):
    """
    First bool is if you should continue computation, second bool is if they are the same:
    """
    if sol_1 is None and sol_2 is None:
        return True, True  # If both are None → considered equal → return True, True
    elif sol_1 is not None and sol_2 is not None:
        return False, None # If both are not None → proceed to actual comparison → return False, None
    else:
        return True, False # If one is None → considered unequal → return True, False

def ik_sol_is_close(sol_1, sol_2):

    found_none, success = check_for_none(sol_1, sol_2)
    if found_none: return success

    success = np.allclose(sol_1, sol_2)
    if not success:
        print(f"[ERROR] IK solutions do not match")
        print(f"ik_sol_1: ", np.round(sol_1,6))
        print(f"ik_sol_2: ", np.round(sol_2,6))
    return success

def fk_sol_is_close(sol_1, sol_2):
    found_none, success = check_for_none(sol_1, sol_2)
    if found_none: return success

    return pose_is_close(sol_1, sol_2)

    # success = np.allclose(sol_1, sol_2)
    # if not success:
    #     print(f"[ERROR] FK solutions do not match")
    #     print(f"fk_sol_1: ", np.round(sol_1,6))
    #     print(f"fk_sol_2: ", np.round(sol_2,6))
    # return success

def pose_is_close(pose_1, pose_2):
    """
    I thought I may need to do quaternion difference, but because it's small angles, you can just compare directly!

    This may not be the case!!! right near the edges, it may return a different euler angle....
    """
    success = np.allclose(pose_1, pose_2)
    if not success:
        print(f"[ERROR] Poses do not match")
        print(f"pose_1: ", np.round(pose_1,6))
        print(f"pose_2: ", np.round(pose_2,6))

        # See this function here.. for ref
        # from bam_ros_utils.geometry import pose_close
        print("Trying to compare angles directly")
        rpy_1 = pose_1[3:]
        rpy_2 = pose_2[3:]

        success = quats_equal_up_to_sign(rpy_1, rpy_2)

        if success:
            print("Success")
        else:
            print("Failed")

    return success
