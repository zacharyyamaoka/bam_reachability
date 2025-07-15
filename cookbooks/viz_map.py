from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.visualizer.aligned_slicer import Open3DMapViewer
from bam_reachability.visualizer.colorize_map import colorize_reachability


map_path = "/home/bam/python_ws/bam_reachability/maps/mock/Mock_table_1.2x1.2x1.0_0.10_90x45x360_map_13_jul_2025.pkl"
map = ReachabilityMap.load(map_path)
colors = colorize_reachability(map, show_histogram=False)


Open3DMapViewer(map.positions, colors=colors).run()