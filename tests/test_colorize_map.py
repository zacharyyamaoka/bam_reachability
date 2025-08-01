from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.visualization.open3d_map_viewer import Open3DMapViewer
from bam_reachability.visualization.colorize_map import bin_values, plot_bins, colorize_reachability
import numpy as np

def test_bin_scores():
    scores = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
    expected_bin_ids = np.array([1, 1, 2, 2, 3, 3, 3, 4, 4, 5])
    expected_bin_normalized = np.array([0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.5, 0.75, 0.75, 1.0])

    n_bins = 5
    binned_scores, bin_edges = bin_values(scores, first_edge=0.2, last_edge=0.999, n_bins=n_bins, normalize=False)

    # plot_bins(scores, bin_edges)

    assert np.all(binned_scores == expected_bin_ids)
    assert len(binned_scores) == len(scores)
    assert len(np.unique(binned_scores)) == n_bins
    assert np.max(binned_scores) == n_bins
    assert np.min(binned_scores) == 1

    binned_scores, bin_edges = bin_values(scores, first_edge=0.2, last_edge=0.999, n_bins=n_bins, normalize=True)
    assert np.all(binned_scores == expected_bin_normalized)
    assert len(binned_scores) == len(scores)
    assert len(np.unique(binned_scores)) == n_bins
    assert np.max(binned_scores) == 1.0
    assert np.min(binned_scores) == 0.0

if __name__ == "__main__":
    test_bin_scores()
