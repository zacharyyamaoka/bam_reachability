#!/usr/bin/env python3

"""
*** this requires ROS to be setup on computer

The most minimal setup to test connection is:

Launch Moveit:
ros2 launch bam_core_bringup moveit.launch.py robot:=ur gripper:=none plugin:=none solver:=kdl rviz:=true

Launch Joint State Buffer:
ros2 run bam_ros_utils joint_state_buffer

---

A simpler thing would be to just require the demo to run

The moveit.launch.py is quite simple as well though, basically doing the same thing!

ros2 launch bam_arm_moveit_config demo.launch.py 


The issue is that the other launch files don't bring up the robot descriptions!!!

I do really like the idea of just needing to launch the demo moveit.. that is how I do it in testing. Ok I need one for UR then though

# Using Same URDF as used in system, You could also use a simpler one...

urdf_simple="/home/bam/public_ws/src/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro"

urdf_path="$HOME/public_ws/src/Universal_Robots_ROS2_GZ_Simulation/ur_simulation_gz/urdf/ur_bam_eef.urdf.xacro"
save_path="$HOME/bam_ws/src/bam_ros/bam_descriptions/robots/ur5e.urdf"
ros2 run xacro xacro "$urdf_path" ur_type:=ur5e name:=ur5e gripper:=none > "$save_path"

ros2 launch moveit_setup_assistant setup_assistant.launch.py 

Set Group Names to the same as other moveit packages.. I can just copy paste this afterwarsd I think...

-- I shouldn't care about the gripper btw when I do this, the gripper can be attached but it shouldn't matter... really...

Ok I like the Idea of having a UR configuration seperatly... thats good!


I am very sure that moveit demo will always be working... cause thats just the baseline... for moveit

ros2 launch ur5e_moveit_config demo.launch.py
"""

# ROS
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from tf_transformations import euler_from_quaternion

# BAM
from bam_descriptions import get_robot_params
from bam_moveit_client import MoveItClient
from bam_ros_utils.msgs import get_joint_state, get_pose_stamped
from bam_ros_utils.np_helper import to_numpy_xyzrpy
from bam_ros_utils.geometry import move_relative

# PYTHON
from typing import Tuple
import os
import numpy as np
import time


class MoveItKinWrapper():
    
    def __init__(self, arm="ur", node=None, spin=True, ns=f"bam_{os.environ.get('ROBOT_ID')}"):

        rp = get_robot_params(arm)

        if node is None:
            node = Node('generic_node', namespace=ns)
        
        node.get_logger().set_level(LoggingSeverity.WARN)
        self.moveit_client = MoveItClient(node, rp.move_group_arm, rp.base_link, rp.ik_tip, spin, mock_js=True)
        # self.rate = node.create_rate(50)  # 10 Hz = 0.1 sec You cannot use this beacuse nothing calls it!

        self.robot_params = rp
    
    def IK(self, pose: np.ndarray, seed: np.ndarray = None )-> Tuple[bool, np.ndarray]:
        # Assumes poses are with respect to rp.base_link
        # Transform pose from tool0 (z sticks out of joint 6) to ee_link
        pose_msg = get_pose_stamped(pose[:3], pose[3:], self.robot_params.base_link)
        pose_msg = move_relative(pose_msg, rpy=[np.pi/2, -np.pi/2, 0], local=True)

        if seed == None:
            seed = self.robot_params.ZERO_JOINT_STATE
        (r_ik, joint_state) = self.moveit_client.compute_ik(pose_msg, seed) # won't be good for kdl...
        if not r_ik.success: return False, None

        joint_positions = np.array(joint_state.position)
        return True, joint_positions

    def FK(self, joint_positions: np.ndarray)-> Tuple[bool, np.ndarray]:

        js = get_joint_state(self.robot_params.joint_names, joint_positions.tolist())
        (r_fk, pose_stamped) = self.moveit_client.compute_fk(js)

        if not r_fk.success: return False, None

        # Transform pose from ee_link to tool0 
        pose_stamped = move_relative(pose_stamped, rpy=[-np.pi/2, 0, -np.pi/2], local=True)

        pose = to_numpy_xyzrpy(pose_stamped)

        return True, pose


if __name__ == '__main__':

    # TO RUN:
    # ros2 launch ur5e_moveit_config demo.launch.py

    """
    Bugs I found when making this:

    1. You cannot directly compare joint positions as IK has 8 possible ones, and it only returns 1. Better to compare the xyzrpy pose
    2. Sometimes a solution can exist but not the one your choosing! The orignal q is in joint 4, but joint 2 is nothing! I have added
        a ur5ef IK solution that returns the next valid solution for this case.

    [10] FK mismatch
    q: [ 0.318 -2.304  0.108  0.136  1.969  0.325]
    q_pose: [-0.475 -0.057  0.924 -0.305 -0.951 -2.093]
    ik_sol : [0.817 0.233 0.063 3.142 0.    1.571]

    ---- Terminal Log ----

    [move_group-3] Analytic IK returned 6 raw solutions:
    [move_group-3]   Solution 0: [2.85289, 4.82434, 0.68329, 5.65184, 0.97021, 4.12277]
    [move_group-3]   Solution 1: [2.85289, 5.47908, 5.59990, 0.08049, 0.97021, 4.12277]
    [move_group-3]   Solution 2: None
    [move_group-3]   Solution 3: None
    [move_group-3]   Solution 4: [0.31796, 3.97931, 0.10793, 0.13645, 1.96875, 0.32489]
    [move_group-3]   Solution 5: [0.31796, 4.08291, 6.17525, 0.24872, 1.96875, 0.32489]
    [move_group-3]   Solution 6: [0.31796, 4.09269, 0.38550, 2.88710, 4.31444, 3.46648]
    [move_group-3]   Solution 7: [0.31796, 4.46251, 5.89769, 3.28827, 4.31444, 3.46648]
    [move_group-3] 
    [move_group-3] [INFO] [1749775613.089939258] [move_group.moveit.moveit.kinematics.ur_kinematics_plugin]: Joint limits:
    [move_group-3]   Joint 0: min=-3.14159, max=3.14159
    [move_group-3]   Joint 1: min=-3.14159, max=3.14159
    [move_group-3]   Joint 2: min=-3.14159, max=3.14159
    [move_group-3]   Joint 3: min=-3.14159, max=3.14159
    [move_group-3]   Joint 4: min=-3.14159, max=3.14159
    [move_group-3]   Joint 5: min=-3.14159, max=3.14159
    [move_group-3] 
    [move_group-3] [INFO] [1749775613.089955058] [move_group.moveit.moveit.kinematics.ur_kinematics_plugin]: 
    [move_group-3] After wrapping 6 solutions:
    [move_group-3]   Solution 0: [2.85289, -1.45885, 0.68329, -0.63134, 0.97021, -2.16042]
    [move_group-3]   Solution 1: [2.85289, -0.80410, -0.68329, 0.08049, 0.97021, -2.16042]
    [move_group-3]   Solution 2: None
    [move_group-3]   Solution 3: None
    [move_group-3]   Solution 4: [0.31796, -2.30388, 0.10793, 0.13645, 1.96875, 0.32489]
    [move_group-3]   Solution 5: [0.31796, -2.20028, -0.10793, 0.24872, 1.96875, 0.32489]
    [move_group-3]   Solution 6: [0.31796, -2.19050, 0.38550, 2.88710, -1.96875, -2.81670]
    [move_group-3]   Solution 7: [0.31796, -1.82067, -0.38550, -2.99492, -1.96875, -2.81670]


    """
    
    import time

    rclpy.init(args=None)

    K = MoveItKinWrapper(arm="ur", ns="")
    time.sleep(1)

    lower_bounds = K.robot_params.lower_limits
    upper_bounds = K.robot_params.upper_limits

    num_tests = 100
    success_count = 0

    for i in range(num_tests):
        print(f"iter {i}/{num_tests}")
        q_rand = np.random.uniform(lower_bounds, upper_bounds)
        fk_success, q_rand_pose = K.FK(q_rand)
        if not fk_success:
            print(f"[{i}] FK q_rand failed")
            continue

        ik_success, ik_sol = K.IK(q_rand_pose)
        if not ik_success:
            print(f"[{i}] IK q_rand_pose failed")
            continue

        fk_success, fk_sol = K.FK(ik_sol)
        if not fk_success:
            print(f"[{i}] FK ik_sol failed")
            continue

        # You cannot directly compare as IK has 8 possible solutions!
        if np.allclose(q_rand_pose, fk_sol):
            success_count += 1
        else:
            print(f"[{i}] FK mismatch\n  q: {np.round(q_rand,3)}\n q_pose: {np.round(q_rand_pose,3)}\n  ik_sol : {np.round(fk_sol,3)}")
            break

    print(f"\n{success_count}/{num_tests} solutions matched")

    
    rclpy.shutdown()
    exit(0)
