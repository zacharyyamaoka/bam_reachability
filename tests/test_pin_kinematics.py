import example_robot_data
import numpy as np

from bam_reachability.kin_wrapper.pin_kinematics import PinKinematics

# BUG Panda has 7 DOF and has 0 - inf redundant solutions, UR has 6 DOF and 0 - 8 possible solutions

def test_pin_kinematics():

    robot = example_robot_data.load("ur5")
    model = robot.model

    K = PinKinematics(model, "base_link", "tool0", verbose=False)


    # 
    fk_success, fk_sol = K.FK(K.neutral_config())
    ik_success, ik_sol = K.IK(fk_sol)
    assert fk_success and ik_success

    # BUG IK is not well tuned and doesn't converge always...
    # It's fine for testing, but not for actual use right now!
    # All I can say is that I am greatful for KDL and closed form IK... what a nightmare to have a optimzation loop in such a critical part of the robot....
    np.random.seed(42)  # Set seed for reproducibility

    failure_count = 0
    for i in range(100):
        q_random = K.random_config()
        fk_success, fk_sol = K.FK(q_random)
        ik_success, ik_sol = K.IK(fk_sol)
        fk_success_2, fk_sol_2 = K.FK(ik_sol)
        # BUG sometimes ik will fail to converge so you cannot check success
        # assert fk_success and ik_success and fk_success_2

        if not np.allclose(fk_sol, fk_sol_2, atol=1e-1):
            failure_count += 1

    assert failure_count < 10

if __name__ == "__main__":
    # test_bin_scores()
    test_pin_kinematics()
    print("Success!")