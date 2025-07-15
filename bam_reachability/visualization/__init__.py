from bam_reachability.visualization.open3d_map_viewer import Open3DMapViewer
from bam_reachability.visualization.colorize_map import colorize_reachability, colorize_inconsistency

try:
    from bam_reachability.visualization.meshcat_map_viewer import MeshcatMapViewer
except ImportError:
    print("must install pin_utils")
