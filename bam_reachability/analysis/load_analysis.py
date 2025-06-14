#!/usr/bin/env python3

from bam_reachability.visualizer import AlignedSlicer, colorize_map, colorize_inconsistency
from bam_reachability.reachability_map import ReachabilityMap


import os

# helper function is file is local
# base_dir = os.path.dirname(os.path.abspath(__file__))
# file_path = os.path.join(base_dir, 'mock_map_13_jun_2025.pkl')

# file_path = "/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur/ur_moveit_map_13_jun_2025.pkl"
file_path = "/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur_offset_wrist_map_13_jun_2025.pkl"

map = ReachabilityMap.load(file_path)
print("Frames: ", map.frames.shape)
print("Map: ", len(map.ik_map))

# points, colors = colorize_map(map, histogram=True)
points, colors = colorize_inconsistency(map)


print(points.shape, colors.shape)
step = 0.2
slicer = AlignedSlicer(points, step, colors, hide_alpha=False)

slicer.run()
