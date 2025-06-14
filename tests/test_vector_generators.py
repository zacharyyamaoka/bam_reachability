
#!/usr/bin/env python3

from bam_reachability.generators import generate_orthonormal_vectors 
import numpy as np

def test_generate_orthonormal_vectors():
    vectors = generate_orthonormal_vectors(axis=(0,0,-1), angle_step_rad=np.deg2rad(45))
    assert vectors.shape[0] == 8

    vectors = generate_orthonormal_vectors(axis=(0,0,-1), angle_step_rad=np.deg2rad(90))
    assert vectors.shape[0] == 4

    vectors = generate_orthonormal_vectors(axis=(0,0,-1), angle_step_rad=np.deg2rad(180))
    assert vectors.shape[0] == 2

    vectors = generate_orthonormal_vectors(axis=(0,0,-1), angle_step_rad=np.deg2rad(360))
    assert vectors.shape[0] == 1



if __name__ == "__main__":
    test_generate_orthonormal_vectors()
    print("All tests passed")
