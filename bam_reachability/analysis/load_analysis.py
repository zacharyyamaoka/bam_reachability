#!/usr/bin/env python3

from bam_reachability.visualizer import AlignedSlicer, colorize_map, colorize_inconsistency, MeshcatMapViewer
from bam_reachability.reachability_map import ReachabilityMap
from bam_descriptions import get_robot_params

import os

arm_name = "ur"
show_histogram = True
file_path ="/home/bam/bam_ws/src/bam_plugins/bam_reachability/maps/ur/ur_offset_table_1.0x0.5x0.2_0.10_85x42x360_map_15_jun_2025.pkl"

map = ReachabilityMap.load(file_path)
print("Frames: ", map.frames.shape)
print("Map: ", len(map.ik_map))

points, colors = colorize_map(map, histogram=show_histogram)
# points, colors = colorize_inconsistency(map, show_histogram)


print(points.shape, colors.shape)

# MESHCAT
rp = get_robot_params(arm_name) 
print("Loaded Robot Params for: ", rp.name)
meshcat_client = MeshcatMapViewer.from_robot_param(map, colors, rp)

# O3D
# step = 0.2
# slicer = AlignedSlicer(points, step, colors, hide_alpha=False)
# slicer.run()
