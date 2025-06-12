#!/usr/bin/env python3

import numpy as np
import open3d as o3d
from transforms3d.quaternions import rotate_vector, axangle2quat
from typing import Tuple

"""
Inspired by:
https://github.com/compas-dev/compas_fab/blob/main/src/compas_fab/robots/reachability_map/vector_generators.py

- Right now using transforms3d library, could also mabye us tf_transforms at some point for less low level, this seems to be working though!

Implemented by Chat GPT

"""


def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm > 0 else v


def orthogonal_vector(v):
    """Find any unit vector orthogonal to v"""
    if np.allclose(v, [0, 0, 1]):
        return np.array([1, 0, 0])
    return normalize(np.cross(v, [0, 0, 1]))


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

    return np.array(vectors)


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
    vectors = generate_deviation_vectors(
        axis=(-1, 0, 0),  # z-axis
        max_angle_rad=np.deg2rad(90),  # 90 degree cone
        step_angle_rad=np.deg2rad(30)
    )
    visualize_vectors(vectors)
    print(vectors.shape)

    mask = mask_vectors_by_angle(vectors, [0,0,1], np.deg2rad(45) )
    print(mask.shape)
    print(np.sum(mask))
    visualize_vectors(vectors[mask])