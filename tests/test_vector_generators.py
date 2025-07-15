
#!/usr/bin/env python3

from bam_reachability.generators import generate_orthonormal_vectors, generate_deviation_vectors, visualize_vectors, mask_R_list_by_angle
import numpy as np
from transforms3d.euler import mat2euler, euler2mat
import pytest
from bam_reachability.reachability_map import ReachabilityMap

def test_transforms():
    R = np.eye(3)
    rpy = mat2euler(R)
    assert np.allclose(rpy, [0.0, 0.0, 0.0])

def assert_unique_vectors(vectors):
    for i, v1 in enumerate(vectors):
        for j, v2 in enumerate(vectors):
            if i != j:
                assert not np.allclose(v1, v2), f"Vectors at indices {i} and {j} are not unique: {v1}, {v2}"

def test_view_generators():

    # if max_angle_rad is 0, then we should only get the axis vector (case were step_angle_rad is non zero)
    vectors = generate_deviation_vectors(
        axis=(0, 0, -1), 
        max_angle_rad=np.deg2rad(0),  
        step_angle_rad=np.deg2rad(30)
    )
    assert len(vectors) == 1 

    # if max_angle_rad is 0, then we should only get the axis vector (case were step_angle_rad is 0)
    vectors = generate_deviation_vectors(
        axis=(0, 0, -1), 
        max_angle_rad=np.deg2rad(0), 
        step_angle_rad=np.deg2rad(0) # Does 0 deg step break it?
    )
    assert len(vectors) == 1 

    # if max_angle_rad is 0, then we should only get the axis vector (case were step_angle_rad is 0)
    vectors = generate_deviation_vectors(
        axis=(0, 0, -1), 
        max_angle_rad=np.deg2rad(90), 
        step_angle_rad=np.deg2rad(45) # Does 0 deg step break it?
    )
    assert len(vectors) == 14
    assert any(np.allclose(v, [0, 0, -1]) for v in vectors)
    assert_unique_vectors(vectors)


    axis = np.array([0.5, 0.5, 1]) 
    unit_axis = axis / np.linalg.norm(axis)
    vectors = generate_deviation_vectors(
        axis=axis, 
        max_angle_rad=np.deg2rad(90), 
        step_angle_rad=np.deg2rad(45) # Does 0 deg step break it?
    )
    assert len(vectors) == 14
    assert not any(np.allclose(v, axis) for v in vectors)
    assert any(np.allclose(v, unit_axis) for v in vectors)
    assert_unique_vectors(vectors)

    # visualize_vectors(vectors)


def test_generate_orthonormal_vectors():
    vectors = generate_orthonormal_vectors(axis=(0,0,-1), angle_step_rad=np.deg2rad(45))
    assert vectors.shape[0] == 8

    vectors = generate_orthonormal_vectors(axis=(0,0,-1), angle_step_rad=np.deg2rad(90))
    assert vectors.shape[0] == 4

    vectors = generate_orthonormal_vectors(axis=(0,0,-1), angle_step_rad=np.deg2rad(180))
    assert vectors.shape[0] == 2

    vectors = generate_orthonormal_vectors(axis=(0,0,-1), angle_step_rad=np.deg2rad(360))
    assert vectors.shape[0] == 1


def test_mask_R_list_by_angle():
    orientations = np.array([euler2mat(0, 0, 0), euler2mat(0, 0, np.pi), euler2mat(0, np.pi/2, 0), euler2mat(0, -np.pi/2, 0), euler2mat(np.pi/2, 0, 0), euler2mat(-np.pi/2, 0, 0)])

    mask = mask_R_list_by_angle(orientations, [0, 0, 1], np.deg2rad(90))
    assert np.all(mask == [1, 1, 1, 1, 1, 1])

    mask = mask_R_list_by_angle(orientations, [0, 0, 1], np.deg2rad(80))
    assert np.all(mask == [1, 1, 0, 0, 0, 0])

    orientations = np.array([euler2mat(0, 0, 0), euler2mat(0, 0, np.pi), euler2mat(0, np.pi, 0), euler2mat(0, -np.pi, 0), euler2mat(np.pi, 0, 0), euler2mat(-np.pi, 0, 0)])
    mask = mask_R_list_by_angle(orientations, [0, 0, 1], np.deg2rad(180))
    assert np.all(mask == [1, 1, 1, 1, 1, 1])

    with pytest.raises(AssertionError):
        mask = mask_R_list_by_angle(orientations, [0, 0, 1], np.deg2rad(360))

def test_mask_R_list_by_angle_diff_vector():
    orientations = np.array([euler2mat(0, 0, 0), euler2mat(0, 0, np.pi), euler2mat(0, np.pi/2, 0), euler2mat(0, -np.pi/2, 0), euler2mat(np.pi/2, 0, 0), euler2mat(-np.pi/2, 0, 0)])

    # any rotation around x or y shouldn't change the distance which is 90!
    mask = mask_R_list_by_angle(orientations, [1, 0, 0], np.deg2rad(90))
    # I orginally though it would be [1, 1, 0, 0, 1, 1]
    # but actually rotating around the y axis moves z directly towards the target vector or in the opposite direction!
    # so its [1, 1, 1, 0, 1, 1]
    # [1.57079633 1.57079633 0.         3.14159265 1.57079633 1.57079633]
    assert np.all(mask == [1, 1, 1, 0, 1, 1])

    mask = mask_R_list_by_angle(orientations, [1, 0, 0], np.deg2rad(80))
    assert np.all(mask == [0, 0, 1, 0, 0, 0])

def test_mask_R_map():
    map = ReachabilityMap.load("/home/bam/python_ws/bam_reachability/maps/ur/ur5e/ur5e_table_1.2x1.2x1.0_0.10_90x45x360_15_jul_2025.pkl")

    hemisphere_mask = mask_R_list_by_angle(map.orientations, [0, 0, -1], np.deg2rad(90))
    four_dof_mask = mask_R_list_by_angle(map.orientations, [0, 0, -1], np.deg2rad(5))
    assert np.sum(hemisphere_mask) == len(hemisphere_mask)
    assert np.sum(four_dof_mask) == 1

if __name__ == "__main__":
    # test_generate_orthonormal_vectors()
    # test_view_generators()
    # test_transforms()
    test_mask_R_list_by_angle()
    test_mask_R_list_by_angle_diff_vector()
    test_mask_R_map()
    print("All tests passed")
