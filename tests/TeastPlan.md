

### CLI 


I will test for the mock kinematics in here.

- I will implement the moveit/pin kinematics here with the * that they require thoose packages

- Tests for thoose should live in BAM_tests IMO, b/c they require automated launches, this is just a python package...


--

There are 4 things that I need to verify


1. That the IK/FK function is consitent

    q_ik = IK(pose)
    pose_fk = FK(q_ik)
    pose_fk = pose

2. That the IK/FK call is repeatable/Deterministic (You always get the same result)

    q_ik_1 = IK(pose)
    q_ik_2 = IK(pose)
    pose_fk_1 = FK(pose)
    pose_fk_2 = FK(pose)

    q_ik_1 = q_ik_2
    pose_fk_1 = pose_fk_2

3. That it is the same as before (in case DH params where changed etc.)

    q_ik = IK(pose_old)
    q_ik = q_old

    pose_fk = FK(q_old)
    pose_fk = pose_old

4. That is it consistent with another map (ie. Moveit vs Pinnochio)


    q_ik_moveit = IK_moveit(pose)
    pose_fk_moveit = FK_moveit(q_ik_moveit)
    q_ik_pin = IK_pin(pose)
    pose_fk_pin = FK_pin(q_ik_pin)

    q_ik_moveit = q_ik_pin
    pose_fk_pin = pose_fk_moveit

These are actually all special cases of just comparing a map to another map

1. Is done inside the map
2. Is comparing a map to itself (immedietly, and as many times as you want)
3. Is comparing a map to an older map
4. Is comparing a map to a map generated for same frames by a different kinematic function

- This can all be done via the simple map datastructure, which should lend it self to resuability and ease of maintence
- I can create maps of different sizes, for just fasting checking nothing has changed, vs more intensive coverage