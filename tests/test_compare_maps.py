from bam_reachability.analysis.compare_maps import compare_maps
from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.generators import TablePoseGenerator
from bam_reachability.utils.math_utils import get_matrix
from bam_reachability.kin_wrapper import MockKinWrapper
import numpy as np

# I want to verify that when maps are the same it returns success...
# when the maps are different, it returns failure!

def test_compare_maps():

    # 1. Create Generator
    generator = TablePoseGenerator(
    pose_matrix = get_matrix(([0, 0.3, 0.1], [-0.05, 0, 0])),
    scale = (0.75, 0.4, 0.1),
    xyz_step = 0.5,
    hemisphere_angle = np.deg2rad(45),
    view_step = np.deg2rad(45),
    rotation_step = np.deg2rad(180),
    viz = False,
    )

    positions, orientations = generator.generate(viz=False)

    # 2. Create IK/FK functions
    K1 = MockKinWrapper(L1=0.3, L2=0.3, seed=1, random_ik_fail=True)
    K2 = MockKinWrapper(L1=0.3, L2=0.3, seed=2, random_ik_fail=True)
    K3 = MockKinWrapper(L1=0.4, L2=0.4, seed=1, random_ik_fail=True)
    
    # 3. Create Map and save result
    map_1 = ReachabilityMap(positions, orientations, K1.IK, K1.FK)
    map_1.calculate_ik_fk_map()

    map_2 = ReachabilityMap(positions, orientations, K2.IK, K2.FK)
    map_2.calculate_ik_fk_map()

    map_3 = ReachabilityMap(positions, orientations, K3.IK, K3.FK)
    map_3.calculate_ik_fk_map()

    # 4. Compare Maps
    same, summary = compare_maps(map_1, map_1)
    assert same

    same, summary = compare_maps(map_1, map_2)
    assert not same

    same, summary = compare_maps(map_1, map_3)
    assert not same


if __name__ == "__main__":
    test_compare_maps()