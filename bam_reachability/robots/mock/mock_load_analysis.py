#!/usr/bin/env python3

from bam_reachability.visualizer import AlignedSlicer
from bam_reachability.reachability_map import ReachabilityMap
import os

base_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(base_dir, 'mock_map.pkl')

reachability = ReachabilityMap.load(file_path)
print("Frames: ", reachability.frames.shape)
print("Map: ", len(reachability.map))

points, colors = reachability.colorize(histogram=False)
print(points.shape, colors.shape)
step = 0.1
slicer = AlignedSlicer(points, step, colors, hide_alpha=True)

slicer.run()
