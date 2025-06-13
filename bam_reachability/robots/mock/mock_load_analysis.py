#!/usr/bin/env python3

from bam_reachability.visualizer import AlignedSlicer, colorize_map
from bam_reachability.reachability_map import ReachabilityMap


import os

base_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(base_dir, 'mock_map_12_jun_2025.pkl')

map = ReachabilityMap.load(file_path)
print("Frames: ", map.frames.shape)
print("Map: ", len(map.ik_map))

points, colors = colorize_map(map, histogram=True)
print(points.shape, colors.shape)
step = 0.1
slicer = AlignedSlicer(points, step, colors, hide_alpha=False)

slicer.run()
