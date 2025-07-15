#!/usr/bin/env python3

"""
    Inspired by:
    https://github.com/compas-dev/compas_fab/blob/main/src/compas_fab/robots/reachability_map/vector_generators.py

    - Right now using transforms3d library, could also mabye us tf_transforms at some point for less low level, this seems to be working though!

    Implemented by Chat GPT

    One problem is that I need poses for the IK, not direction vectors...

    Convert direction vectors into eulers? or...


    How is it done in grasping?

    https://github.com/GouMinghao/rgb_matters/blob/main/rgbd_graspnet/data/utils/gen_label.py


    They read in the grasps in free 6DOF space and associate it with the closet discrete direction and angle

    https://github.com/GouMinghao/rgb_matters/blob/2e176728f6dae5e878447240ce6a2423bf770f36/rgbd_graspnet/data/utils/gen_label.py#L127C1-L130C74
    Rs = grasp.rotation_matrices
    # widths = np.hstack([widths,g_obj['widths']]) # width
    points = grasp.translations
    views_index, angles_index = get_towards_and_angles(Rs, anchor_matrix)

    see: get_towards_and_angles
    https://github.com/GouMinghao/rgb_matters/blob/main/rgbd_graspnet/data/utils/view_rotation.py#L12


    see: anchor_matrix = generate_matrix()
    https://github.com/GouMinghao/rgb_matters/blob/main/rgbd_graspnet/data/utils/generate_anchor_matrix.py#L119


    Ok what I want to do for the IK testing is a bit different beacuse I don't care as much about the angle vector view, I can always set that to be zero!

    See how COMPAS generates frames: https://github.com/compas-dev/compas_fab/blob/005e2680385b80c7f0bd4dfd30d080fd2c135467/tests/robots/test_reachability.py#L42

"""

import numpy as np
import open3d as o3d
from transforms3d.quaternions import rotate_vector, axangle2quat

from typing import Tuple, List


def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm > 0 else v


def orthogonal_vector(v):
    """Find any unit vector orthogonal to v"""
    # Pick a vector not collinear with v

    # use x axis [1, 0, 0] for cross product, unless its close, then use y axis [0, 1, 0]

    if np.allclose(np.abs(v), [1, 0, 0]):
        other = np.array([0, 1, 0])
    else:
        other = np.array([1, 0, 0])

    return normalize(np.cross(v, other))

def orthogonal_to_two_vectors(v1, v2):
    """Find a vector orthogonal to two given vectors"""
    v = np.cross(v1, v2)
    norm_v = np.linalg.norm(v)
    if norm_v > 1e-8:
        return v / norm_v

    else:
        # Fallback if v1 and v2 are (nearly) colinear
        return orthogonal_vector(v1)

def generate_orthonormal_vectors(axis, angle_step_rad, start_vector=None):
    """
    Generate vectors orthonormal to the given axis, spaced by angle_step_rad.

    Parameters
    ----------
    axis : array-like (3,)
        The reference axis (e.g., z-axis of a pose).
    angle_step_rad : float
        The angle step in radians (e.g., np.deg2rad(90)).
    start_vector : array-like (3,), optional
        A preferred starting vector (not aligned with axis). If None, one is chosen automatically.

    Returns
    -------
    List of np.ndarray
        A list of unit vectors orthonormal to the given axis.
    """
    axis = normalize(np.array(axis))

    if start_vector is not None:
        sv = normalize(np.array(start_vector))
        proj = sv - np.dot(sv, axis) * axis  # remove component along axis, in case it's not orthogonal
        x0 = normalize(proj)
    else:
        x0 = orthogonal_vector(axis)

    vectors = []
    num_steps = int(np.ceil(2 * np.pi / angle_step_rad))
    for i in range(num_steps):
        angle = i * angle_step_rad
        q = axangle2quat(axis, angle)
        x = normalize(rotate_vector(x0, q))
        vectors.append(x)

    return np.array(vectors)


def generate_deviation_vectors(axis=[0,0,1], max_angle_rad=np.deg2rad(180), step_angle_rad=np.deg2rad(15)):
    """
    Generate unit vectors that deviate from the given axis direction
    by up to max_angle_rad, in step_angle_rad increments.

    Parameters:
        axis (array-like): Primary axis direction (e.g. [0, 0, 1]).
        max_angle_rad (float): Maximum deviation angle (in radians).
        step_angle_rad (float): Angular step size (in radians).

    Returns:
        List of np.ndarray unit vectors.
    """
    axis = normalize(np.array(axis))
    vectors = [axis]

    if max_angle_rad == 0:
        return vectors

    ortho = orthogonal_vector(axis)
    alphas = np.arange(step_angle_rad, max_angle_rad + 1e-5, step_angle_rad)

    for alpha in alphas:
        r = np.sin(alpha)
        num_points = max(4, int(2 * np.pi * r / step_angle_rad))

        betas = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        q_alpha = axangle2quat(ortho, alpha)

        for beta in betas:
            q_beta = axangle2quat(axis, beta)
            v = rotate_vector(axis, q_alpha)
            v = rotate_vector(v, q_beta)
            vectors.append(normalize(v))

    # Remove duplicates by rounding and using np.unique
    vectors = np.array(vectors)
    rounded = np.round(vectors, decimals=6)  # adjust decimal places as needed
    _, idx = np.unique(rounded, axis=0, return_index=True)

    return vectors[np.sort(idx)]


#
def view_generator(inital_view=[0, 0, 1], hemisphere_angle=np.deg2rad(40), view_step=np.deg2rad(10), rotation_step=np.deg2rad(60)) -> List[np.ndarray]:
    """
    Generate a set of rotation matrices representing different viewing directions and in-plane rotations.

    Parameters
    ----------
    initial_view : list or np.ndarray, shape (3,)
        The base viewing direction. Defaults to [0, 0, 1] (pointing along +Z).
    hemisphere_angle : float
        Maximum deviation angle from the initial_view direction (in radians).
        Defines the cone around `initial_view` to sample directions.
    view_step : float
        Angular step size (in radians) for sampling directions within the hemisphere.
        Smaller values give denser sampling.
    rotation_step : float
        Angular step size (in radians) for sampling in-plane rotations around each view direction.

    Returns
    -------
    List[np.ndarray]
        A list of 3×3 rotation matrices. Each matrix has columns [x_axis, y_axis, z_axis],
        forming a valid right-handed frame.
    """
    # print("Inital View: ", inital_view)
    z_vectors = generate_deviation_vectors(inital_view, hemisphere_angle, view_step)

    R_list = []

    for z in z_vectors:
        start_vector = None
        start_vector = orthogonal_to_two_vectors(z, inital_view)
        x_list = generate_orthonormal_vectors(z, rotation_step, start_vector)
        for x in x_list:
            y = np.cross(z, x)
            R = np.column_stack([x, y, z])  # 3x3 rotation matrix
            R_list.append(R)

    return R_list


def visualize_vectors(vectors, scale=0.2):
    geometries = []
    origin = np.array([0.0, 0.0, 0.0])

    for v in vectors:
        arrow = o3d.geometry.TriangleMesh.create_arrow(
            cylinder_radius=0.005,
            cone_radius=0.01,
            cylinder_height=scale * 0.8,
            cone_height=scale * 0.2,
            resolution=20,
            cylinder_split=4,
            cone_split=1
        )
        # Align arrow (z-up) to vector
        R = get_rotation_matrix_from_z(v)
        arrow.rotate(R, center=(0, 0, 0))
        arrow.translate(origin + v * scale * 0.1)
        arrow.paint_uniform_color([0.2, 0.6, 1.0])
        geometries.append(arrow)

    # Add reference frame
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geometries.append(frame)

    o3d.visualization.draw_geometries(geometries)


def visualize_frames(R_list, scale=0.1, only_z=False):
    """
    Visualize a list of coordinate frames or just their Z axes using Open3D.

    Parameters
    ----------
    R_list : List[np.ndarray] or np.ndarray (3, 3) or (N, 3, 3)
        List of 3x3 rotation matrices or a single one.
    scale : float
        Scale of the coordinate frames or arrows.
    only_z : bool
        If True, only the Z-axis (as an arrow) is visualized.
    """
    geometries = []

    # Normalize input to a list of 3x3 matrices
    if isinstance(R_list, np.ndarray):
        if R_list.ndim == 2:
            R_list = [R_list]
        elif R_list.ndim == 3:
            R_list = list(R_list)
    elif not isinstance(R_list, list):
        R_list = [R_list]

    for R in R_list:
        if only_z:
            arrow = o3d.geometry.TriangleMesh.create_arrow(
                cylinder_radius=0.005,
                cone_radius=0.01,
                cylinder_height=scale * 0.8,
                cone_height=scale * 0.2,
                resolution=20,
                cylinder_split=4,
                cone_split=1
            )
            # Rotate the arrow directly using the full rotation matrix
            arrow.rotate(R, center=(0, 0, 0))
            arrow.translate(R[:, 2] * scale * 0.1)  # small offset along Z
            arrow.paint_uniform_color([0.2, 0.6, 1.0])
            geometries.append(arrow)
        else:
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale)
            frame.rotate(R, center=(0, 0, 0))
            geometries.append(frame)

    # Global reference frame
    ref_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1 * 2)
    geometries.append(ref_frame)

    o3d.visualization.draw_geometries(geometries)

def get_rotation_matrix_from_z(direction):
    """Returns a rotation matrix that aligns z-axis to given direction."""
    direction = normalize(direction)
    z = np.array([0, 0, 1])
    v = np.cross(z, direction)
    c = np.dot(z, direction)
    if np.allclose(v, 0):
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0, -v[2], v[1]],
                   [v[2], 0, -v[0]],
                   [-v[1], v[0], 0]])
    R = np.eye(3) + vx + vx @ vx * ((1 - c) / (np.linalg.norm(v) ** 2))
    return R


def mask_vectors_by_angle(vectors, target_vector, max_angle_rad):
    """Returns mask (1/0) indicating whether each vector lies within max_angle of the target."""
    
    target = normalize(np.array(target_vector))
    vectors = np.array(vectors)
    dots = np.dot(vectors, target)
    dots = np.clip(dots, -1.0, 1.0)  # Numerical safety
    angles = np.arccos(dots)
    mask = angles <= max_angle_rad + 0.1 #add a bit of tolerance, because at 90 for example, it can be cut off
    return mask


if __name__ == "__main__":
    pass




