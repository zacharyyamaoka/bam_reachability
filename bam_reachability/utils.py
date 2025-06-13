#!/usr/bin/env python3

import numpy as np

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

    success = np.allclose(sol_1, sol_2)
    if not success:
        print(f"[ERROR] FK solutions do not match")
        print(f"fk_sol_1: ", np.round(sol_1,6))
        print(f"fk_sol_2: ", np.round(sol_2,6))
    return success

def pose_is_close(pose_1, pose_2):
    """
    I thought I may need to do quaternion difference, but because it's small angles, you can just compare directly!
    """
    success = np.allclose(pose_1, pose_2)
    if not success:
        print(f"[ERROR] Target poses do not match")
        print(f"pose_1: ", np.round(pose_1,6))
        print(f"pose_2: ", np.round(pose_2,6))
    return success
