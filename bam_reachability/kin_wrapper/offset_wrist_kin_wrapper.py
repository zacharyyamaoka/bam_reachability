#!/usr/bin/env python3

# BAM
from bam_reachability.kin_wrapper.kin_wrapper import KinWrapper
from ur_kin_py import OffsetWristKinematics, UR5e_PARAMS, dh_dict_to_list

# PYTHON
from typing import Optional
import numpy as np


class OffsetWristKinWrapper(KinWrapper):

    def __init__(self, name: str, dh_params: list, lower_limits=[-3.14]*6, upper_limits=[3.14]*6, use_tool0=True, verbose=False):

        super().__init__(name)
        self.K = OffsetWristKinematics(dh_params, lower_limits, upper_limits, use_tool0, verbose)


    def IK(self, pose_matrix: np.ndarray, q_seed: Optional[np.ndarray] = None)-> tuple[bool, np.ndarray]:

        q6_des = 0.0
        if q_seed is not None:
            q6_des = q_seed[5]

        ik_success_list, ik_sol_list = self.K.ik(pose_matrix, q6_des)
        ik_success, ik_sol = self.K.select_sol_by_id(ik_success_list, ik_sol_list, 0)

        return bool(ik_success), np.array(ik_sol)


    def FK(self, joint_positions: np.ndarray)-> tuple[bool, np.ndarray]:
        success, pose_matrix = self.K.fk(joint_positions.tolist())

        return success, pose_matrix


class UR5eOffsetWristKinWrapper(OffsetWristKinWrapper):

    def __init__(self, verbose=False):

        dh_list = dh_dict_to_list(UR5e_PARAMS)
        super().__init__('ur5e', dh_list, verbose=verbose)


        self.xacro_path = "/home/bam/python_ws/ur_kin_py/ur_description/urdf/ur5e.xacro"
        self.mesh_directory = "/home/bam/python_ws/ur_kin_py/"


if __name__ == '__main__':
    ur5e = UR5eOffsetWristKinWrapper()
    fk_success, fk_sol = ur5e.FK(np.array([0, 0, 0, 0, 0, 0]))
    ik_success, ik_sol = ur5e.IK(fk_sol)

    print(f"IK success: {ik_success}")
    print(f"IK solution: {ik_sol}")
    print(f"FK success: {fk_success}")
    print(f"FK solution: {fk_sol}")