#!/usr/bin/env python3

import numpy as np
from transforms3d.euler import euler2quat, euler2mat, mat2euler
from transforms3d.quaternions import quat2mat
import os

def make_map_path(curr__file__, arm_name, kinematic_name, generator_name) -> str:
    
    current = os.path.abspath(curr__file__)
    parent1 = os.path.dirname(current)
    parent2 = os.path.dirname(parent1)
    parent3 = os.path.dirname(parent2)

    base_dir = parent3
    file_path = os.path.join(base_dir, 'maps', arm_name, f'{kinematic_name}_{generator_name}_map')
    print("Created file path: ", file_path)

    return file_path

def get_matrix(pose) -> np.ndarray:
    """
    Convert various pose formats to a 4x4 transformation matrix.

    Supported inputs:
    - (6,) array-like: [x, y, z, roll, pitch, yaw]
    - (7,) array-like: [x, y, z, qx, qy, qz, qw]
    - (3, 3) ndarray: Rotation matrix
    - (4, 4) ndarray: Transformation matrix
    - Tuple/list of two arrays: ([x, y, z], [roll, pitch, yaw])
    """
    if isinstance(pose, (list, tuple)) and len(pose) == 2:
        # ([x, y, z], [r, p, y])
        xyz, rpy = np.asarray(pose[0]), np.asarray(pose[1])
        if xyz.shape == (3,) and rpy.shape == (3,):
            return xyzrpy_to_matrix(xyz, rpy)

    if pose.shape == (6,):
        xyz, rpy = pose[:3], pose[3:]
        return xyzrpy_to_matrix(xyz, rpy)

    elif pose.shape == (7,):
        xyz = pose[:3]
        quat = pose[3:]  # [qx, qy, qz, qw]
        mat = np.eye(4)
        mat[:3, :3] = quat2mat(quat)
        mat[:3, 3] = xyz
        return mat

    elif pose.shape == (3, 3):
        mat = np.eye(4)
        mat[:3, :3] = pose
        return mat

    elif pose.shape == (4, 4):
        return pose

    else:
        raise ValueError(f"Unsupported pose format or shape: {pose}")
    
# Helpful utils for working with numpy poses.. for using ROS msgs, see bam_ros_utils.msgs.geometry_converter and 
def xyzrpy_to_matrix(xyz, rpy):
    mat = np.eye(4)
    mat[:3, :3] = euler2mat(*rpy)  # rotation
    mat[:3, 3] = xyz               # translation
    return mat

# See similar func in tf_transformation 
# def euler_from_matrix(matrix, axes='sxyz'):

def matrix_to_rpy(matrix, axes='sxyz'):
    return mat2euler(matrix, axes)

def quats_equal_up_to_sign(rpy1, rpy2, tol=1e-6, verbose=False):
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
    q1 = np.roll(q1, -1)
    q2 = np.roll(q2, -1)

    if np.allclose(q1, q2, atol=tol) or np.allclose(q1, -q2, atol=tol):
        return True
    else:
        if verbose:
            # Compute angle between quaternions
            dot = np.abs(np.dot(q1, q2))  # use absolute for sign invariance
            dot = min(1.0, max(-1.0, dot))  # clamp to valid domain
            angle_rad = 2 * np.arccos(dot)
            angle_deg = np.degrees(angle_rad)
            
            print(f"[ERROR] Rotations do not match within tolerance.")
            print(f"  rpy1: {rpy1}")
            print(f"  rpy2: {rpy2}")
            print(f"  angle difference: {angle_deg:.4f} deg")
        return False
    
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

def ik_sol_is_close(sol_1, sol_2, verbose=False):

    found_none, success = check_for_none(sol_1, sol_2)
    if found_none: return success

    success = np.allclose(sol_1, sol_2)
    if not success and verbose:
        print(f"[ERROR] IK solutions do not match")
        print(f"ik_sol_1: ", np.round(sol_1,6))
        print(f"ik_sol_2: ", np.round(sol_2,6))
    return success

def fk_sol_is_close(sol_1, sol_2, verbose=False):
    found_none, success = check_for_none(sol_1, sol_2)
    if found_none: return success

    return pose_is_close(sol_1, sol_2, verbose)

    # success = np.allclose(sol_1, sol_2)
    # if not success:
    #     print(f"[ERROR] FK solutions do not match")
    #     print(f"fk_sol_1: ", np.round(sol_1,6))
    #     print(f"fk_sol_2: ", np.round(sol_2,6))
    # return success

def pose_is_close(pose_1, pose_2, verbose=False):
    """
    I thought I may need to do quaternion difference, but because it's small angles, you can just compare directly!

    This may not be the case!!! right near the edges, it may return a different euler angle....
    """
    success = np.allclose(pose_1, pose_2)
    if not success:

        # See this function here.. for ref
        # from bam_ros_utils.math.geometry import pose_close
        rpy_1 = pose_1[3:]
        rpy_2 = pose_2[3:]

        success = quats_equal_up_to_sign(rpy_1, rpy_2)

        if not success and verbose:
            print(f"[ERROR] Poses do not match")
            print(f"pose_1: ", np.round(pose_1,6))
            print(f"pose_2: ", np.round(pose_2,6))


    return success
