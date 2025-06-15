#!/usr/bin/env python3

"""
Moveit via RCLPY is very very slow! would take forever to do 26K poses...

Processed 561 / 26620 poses
Processed 562 / 26620 poses
Processed 563 / 26620 poses
Processed 564 / 26620 poses

Its good to get a couple of failures... that means you are towards the bounds:

[WARN] [1749832770.407156886] [generic_node.ik_client]: [Client Response Failure]

  Error Code     : FAILURE(2)
  Error Message  : IKClient.compute() NO_IK_SOLUTION(-31) 

You could sample randomly.. but that is not very good for visualization!

Not clear that this is usable at all!
https://robotics.stackexchange.com/questions/92352/how-to-optimize-service-calls
http://wiki.ros.org/roscpp/Overview/Services#Persistent_Connections
https://github.com/ros2/ros2/issues/1499

# I have seen this even with the exitsting thing... 

Use htop, I see that the process has gone to 100% CPU usage, so its basically running super slow.

Perhaps if I manually throttle it, it will work better...

Python can only use a single thread, so total CPU usage is low

https://discourse.ros.org/t/high-cpu-load-for-simple-python-nodes/28324/21

- I see two poeple here saying that 
    "I have observed this as well. For this reason, I only use rclpy for development, and never for production nodes."
    "Not just CPU overhead when spinning, but memory footprint as well.
    Mirroring Aposhian we usually prototype in python and then convert to C++ for release"

For 100 frames and 20 orientatons, took around 20 minutes to complete!

For possible solution for contiously running node see: https://github.com/ros2/rclpy/issues/1389

Using rclpy wait for future, does not seem to be a very peformant option! good for scripting and small tests, not production!

"""
# ROS
import rclpy

# BAM
from bam_reachability.generators import rectangle_point_generator, generate_deviation_vectors, visualize_vectors
from bam_reachability.visualizer import AlignedSlicer
from bam_reachability.kin_wrapper import MoveItKin

from bam_reachability.reachability_map import ReachabilityMap

# PYTHON
import time
import numpy as np
import os


# 1. Generate Frames
# frames = rectangle_point_generator(scale=(1, 1, 1), step=0.05)
frames = rectangle_point_generator(scale=(1, 1, 1), step=0.2)

orientations = generate_deviation_vectors([0,0,1], np.deg2rad(180), np.deg2rad(60))
print(np.round(orientations))
visualize_vectors(orientations)
print("Frames: ", frames.shape)
print("Orientations: ", orientations.shape)

# 2. Create IK/FK functions

# You must run:
# ros2 launch ur5e_moveit_config demo.launch.py

rclpy.init(args=None)

K = MoveItKin(arm="ur", ns="")
time.sleep(1)

# slicer = AlignedSlicer(frames, step=0.05)
# slicer.run()

reachability = ReachabilityMap(frames, orientations, K.IK, K.FK)


base_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(base_dir, 'ur_moveit_map')

reachability.save(file_path)

rclpy.shutdown()
exit(0)
