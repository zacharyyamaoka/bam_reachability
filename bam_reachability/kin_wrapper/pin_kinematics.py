#!/usr/bin/env python3


# ROS

from geometry_msgs.msg import PoseStamped, Pose

# BAM
from bam_descriptions.robot_params.robot_param import RobotParam
from bam_ros_utils.msgs.geometry_converter import matrix_to_pose_stamped, pose_stamped_to_matrix
from bam_ros_utils.math.geometry import apply_offset

from bam_kinematics_dynamics.six_dof.offset_wrist_math import forward_kinematics_offset_wrist, inverse_kinematics_offset_wrist

# PYTHON
import numpy as np
import pinocchio as pin
from typing import Tuple, Optional


"""

https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html

    Notice that this way to compute the damped pseudo-inverse was chosen mostly because of its simplicity of implementation.
    It is not necessarily the best nor the fastest way, and using a fixed damping factor is not necessarily the best course of action.

This is having problems with convergence!

Ok Problem... for example robot data, its fine to use the world base frame. but more generally you may wanat to use other frames....

One way is to assume that the pose_matrix will always be respect to the world frame, then you can just do

Case 1: (Simplest)
- Assume pose_matrix is always given for ik_joint_frame.
- This is a bad idea! beacuse you want to define the pose_matrix for a meaningful frame (tool0, tcp, etc..)

Case 2:
 - Define pose_matrix for arbitrary ik_tip with respect to world frame
 - T_world_ik_joint = T_world_ik_tip @ T_ik_tip_ik_joint

Case 3: (Most general case, what I have implemented)
 - Define pose_matrix for arbitrary ik_tip with respect to arbitrary base_link
- T_world_ik_joint = T_world_base_link @ T_base_link_ik_tip @ T_ik_tip_ik_joint

"""

def inspect_pin_model(model):
    print(model.names)
    print(model.njoints)
    print(model.nframes)
    print(model.nframes)
    data = model.createData()

    q = pin.neutral(model) 
    print("Joint Positions: ", q)
    pin.forwardKinematics(model, data, q) # set starting position
    print(("[i] {:<24} :  Position xyz".format( "Joint Name")))
    for i in range(model.njoints):
        print(("[{}] {:<24} : {: .4f} {: .4f} {: .4f}".format( i, model.names[i], *data.oMi[i].translation.T.flat )))


class PinKinematics():

    def __init__(self, pin_model: pin.Model, base_link: str, ik_tip_link: str, verbose=False):


        self.model = pin_model
        self.data = self.model.createData()
        self.ik_tip_link = ik_tip_link
        self.base_link = base_link

        self.ik_joint_id = 6 
        self.ik_joint_name = self.model.names[self.ik_joint_id]

        self.verbose = verbose

        q = pin.neutral(self.model) 
        self.T_ik_tip_ik_joint = self.transform_between_frames(q, self.ik_tip_link, self.ik_joint_name)
        self.T_ik_joint_ik_tip = np.linalg.inv(self.T_ik_tip_ik_joint)
        self.T_world_base = self.transform_between_frames(q, "universe", self.base_link)
        self.T_base_world = np.linalg.inv(self.T_world_base)


    def tip_to_joint_frame(self, T_base_ik_tip: np.ndarray) -> np.ndarray:

        T_world_ik_joint = self.T_world_base @ T_base_ik_tip @ self.T_ik_tip_ik_joint
        return T_world_ik_joint

    def joint_to_tip_frame(self, T_world_ik_joint: np.ndarray) -> np.ndarray:

        T_base_ik_tip = self.T_base_world @ T_world_ik_joint @ self.T_ik_joint_ik_tip
        return T_base_ik_tip


    def IK(self, pose_matrix: np.ndarray, seed_q: Optional[np.ndarray] = None) -> tuple[bool, np.ndarray]:
        """ Inverse Kinematics Notes:
            There are two examples of Pinnochio Inverse kinematics:

            1. is just for translation (xyz) - https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/inverse-kinematics-3d.py
            2. The other that we will use is for full 6 dof (xyzrpy) - https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/inverse-kinematics.py

            Even still this takes wayyyy to long to computer for 43K poses for reachability analysis....

        """

        T_world_ik_joint = self.tip_to_joint_frame(pose_matrix)

        # BUG: pin doesn't accept raw numpy matrix, needs its own types
        oMdes = pin.SE3(T_world_ik_joint[:3, :3], T_world_ik_joint[:3, 3])

        q = seed_q
        if q is None:
            q = pin.neutral(self.model)

        eps = 1e-4
        IT_MAX = 100
        DT = 1e-1
        damp = 1e-6

        i = 0
        while True:
            pin.forwardKinematics(self.model, self.data, q) # set starting position
            iMd = self.data.oMi[self.ik_joint_id].actInv(oMdes) # actual T_world_ik_joint vs target T_world_ik_joint
            err = pin.log(iMd).vector  # calculate error (in joint frame)
            if np.linalg.norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break

            # Servo joints to decrease error
            J = pin.computeJointJacobian(self.model, self.data, q, self.ik_joint_id)  # in joint frame
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            v = -J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(self.model, q, v * DT)


            i += 1

        if self.verbose:
            if success:
                print("Convergence achieved!")
            else:
                print(
                    "\n"
                    "Warning: the iterative algorithm has not reached convergence "
                    "to the desired precision"
                )

            print(f"\nresult: {q.flatten().tolist()}")
            print(f"\nfinal error: {err.T}")

        return success, q


    def FK(self, q: np.ndarray) -> tuple[bool, np.ndarray]:

        pin.forwardKinematics(self.model, self.data, q) # set starting position
        T_world_ik_joint = self.data.oMi[self.ik_joint_id]
        T_base_ik_tip = self.joint_to_tip_frame(T_world_ik_joint)
        return True, T_base_ik_tip


    def transform_between_frames(self, q: np.ndarray, frame_a: str, frame_b: str) -> np.ndarray:
        # Update all frame placements
        pin.framesForwardKinematics(self.model, self.data, q)

        # Get frame IDs
        id_a = self.model.getFrameId(frame_a)
        id_b = self.model.getFrameId(frame_b)

        # World (i.e., model root) to frame transforms
        frame_a = self.data.oMf[id_a]
        frame_b = self.data.oMf[id_b]

        T_world_a = np.eye(4)
        T_world_a[:3, :3] = frame_a.rotation
        T_world_a[:3, 3] = frame_a.translation

        T_world_b = np.eye(4)
        T_world_b[:3, :3] = frame_b.rotation
        T_world_b[:3, 3] = frame_b.translation

        # Transform from A to B: T_a_b = inv(T_world_a) @ T_world_b
        T_a_b = np.linalg.inv(T_world_a) @ T_world_b

        return T_a_b

    def neutral_config(self) -> np.ndarray:
        return pin.neutral(self.model)  # or set manually, e.g. np.array([...])

    def random_config(self) -> np.ndarray:
        margin = 1e-3  # avoid values right on the limit

        lower = self.model.lowerPositionLimit.copy()
        upper = self.model.upperPositionLimit.copy()

        # Shrink the joint limits slightly
        lower += margin
        upper -= margin

        # Sample within the reduced bounds
        q = pin.randomConfiguration(self.model, lower, upper)
        q_wrapped = (q + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
        return q_wrapped


if __name__ == "__main__":

    import example_robot_data


    # BUG Panda has many possible solutions!!!

    # robot = example_robot_data.load("panda")
    # model = robot.model
    # K = PinKinematics(model, "panda_link0", "panda_link7", verbose=True)


    # BUG UR also has many solutions actually... 8
    robot = example_robot_data.load("ur5")
    model = robot.model
    # K = PinKinematics(model, "base_link", "tool0", verbose=True)
    # K = PinKinematics(model, "base_link", "ee_link", verbose=True)
    # K = PinKinematics(model, "base_link", "wrist3_link", verbose=True)
    K = PinKinematics(model, "world", "tool0", verbose=True)




    success, pose_matrix = K.FK(K.neutral_config())
    print(pose_matrix)
    success, joint_positions = K.IK(pose_matrix)
    print(success, joint_positions)
    print("")

    q_random = K.random_config()
    success, fk_sol = K.FK(q_random)
    success, ik_sol = K.IK(fk_sol)
    success, fk_sol_2 = K.FK(ik_sol)

    print(q_random)
    print(fk_sol)
    print(ik_sol)
    print(fk_sol_2)
    print(np.round(fk_sol_2 - fk_sol, 3))
    assert np.allclose(fk_sol, fk_sol_2, atol=1e-1)


    # print(joint_positions)
    # print(pose)


    # pose = K.transform_between_frames(K.neutral(),'link_1','link_2')
