#!/usr/bin/env python3
   
import numpy as np
import matplotlib.pyplot as plt
from bam_reachability.reachability_map import ReachabilityMap

def bin_values(values: np.ndarray, first_edge=0.2, last_edge=0.99, n_bins=5, min_value=0.0, max_value=1.0, normalize=True) -> tuple[np.ndarray, list[float]]:
    """
        Bin values into n_bins, controlling the size of the first and last bin by using first_edge and last_edge
    """
    assert first_edge != min_value, "first_edge cannot be min_value, or bin will have no space inside of it!"
    assert last_edge != max_value, "last_edge cannot be max_value, or bin will have no space inside of it!"


    # Step 1: Define bin edges, starting from 0.0

    bin_edges = [min_value]
    n_bins_to_create = n_bins - 2 # for the start and end ones

    edge_step = (last_edge - first_edge) / n_bins_to_create


    for i in range(n_bins_to_create):
        bin_edges.append(first_edge + i * edge_step)
    
    bin_edges.extend([last_edge, max_value])  # always end at 1.0
    # print("Bin Edges: ", bin_edges)

    # Step 2: Assign bin ID to each value
    bin_ids = []

    for val in values:
        for i, edge in enumerate(bin_edges[1:]):
            if val <= edge:
                bin_ids.append(i+1) # +1 because we start at 0
                break

    # print("Bin IDs: ", bin_ids)
    bin_ids = np.array(bin_ids).astype(float)

    if normalize:
        bin_ids -= 1 # start at 0
        bin_ids /= (n_bins - 1)

    return bin_ids, bin_edges

def plot_bins(values: np.ndarray, bin_edges: list[float]):
        
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))

    # It may look a bit funny as it spans [0,1), so first bin starts at 0, last bin includes up to 1.0
    
    # Raw histogram
    # 20 is a nice number because its splits into 0.05 steps 
    raw_counts, raw_bins, _ = axes[0].hist(values, bins=20, color='gray', edgecolor='black')
    axes[0].set_title("Raw Values")
    axes[0].set_xlabel("Values")
    axes[0].set_ylabel("Count")

    # Add bin edge ticks
    axes[0].set_xticks(raw_bins)
    axes[0].set_xticklabels([f"{edge:.2f}" for edge in raw_bins], rotation=45, ha='right')

    # Add counts above bars
    for x, count in zip(raw_bins[:-1], raw_counts):
        if count > 0:
            axes[0].text(x + (raw_bins[1] - raw_bins[0]) / 2, count, str(int(count)),
                        ha='center', va='bottom', fontsize=8)


    hist_counts, _ = np.histogram(values, bins=bin_edges)
    # print("Bin Counts: ", hist_counts)

    bar_positions = np.arange(len(hist_counts))
    bar_labels = [f"{bin_edges[i]:.2f} – {bin_edges[i+1]:.2f}" for i in range(len(hist_counts))]

    bars = axes[1].bar(bar_positions, hist_counts, width=0.8, color='steelblue', edgecolor='black')
    axes[1].bar(bar_positions, hist_counts, width=0.8, color='steelblue', edgecolor='black')
    axes[1].set_xticks(bar_positions)
    axes[1].set_xticklabels(bar_labels, rotation=45, ha='right')
    axes[1].set_title("Binned Values")
    axes[1].set_xlabel("Values")
    axes[1].set_ylabel("Count")

                    # Add counts above bars
    for bar in bars:
        height = bar.get_height()
    
        axes[1].text(bar.get_x() + bar.get_width() / 2, height, str(int(height)),
                    ha='center', va='bottom', fontsize=8)


    plt.tight_layout()
    plt.show()

def colorize_reachability(map: ReachabilityMap, min_threshold=0.2, max_threshold=0.999, n_bins=5, show_histogram=True):
    """
    Return (frames, colors) where colors is (N, 4) RGBA array:
    - Blue = highly reachable
    - Red = poorly reachable
    - Alpha = 0 for unreachable frames

    If histogram=True, shows:
    - Raw score histogram
    - Binned score bar chart with readable bin labels

    """

    scores = np.array([np.mean(info.success) for info in map.ik_map])  # (N,)
    reachable_mask = scores > 0.0

    binned_scores, bin_edges = bin_values(scores, first_edge=0.2, last_edge=0.999, n_bins=n_bins, normalize=True)

    if show_histogram:
        plot_bins(scores[reachable_mask], bin_edges)

    # https://matplotlib.org/stable/users/explain/colors/colormaps.html
    cmap = plt.get_cmap("RdYlGn")
    colors = cmap(binned_scores)
    colors[~reachable_mask, :] = [0.0, 0.0, 0.0, 0.0]

    return colors
    # return map.frames, colors

def colorize_inconsistency(map: ReachabilityMap, show_histogram=True):
    """
    Returns (frames, colors) where:
    - Inconsistent FK/IK frames are colored black (0, 0, 0, 1)
    - Consistent frames are colored green (0, 1, 0, 1)

    Optionally plots a histogram of the number of inconsistent orientations per frame.
    """
    n = map.n_positions
    colors = np.zeros((n, 4))  # RGBA
    inconsistency_counts = np.zeros(n, dtype=int)

    for i in range(n):
        consistent = np.array(map.fk_map[i].consistent).astype(bool)
        n_inconsistent = np.sum(~consistent)
        inconsistency_counts[i] = n_inconsistent

        if n_inconsistent > 0:
            colors[i] = [0.0, 0.0, 0.0, 1.0]  # Black
        else:
            colors[i] = [0.0, 1.0, 0.0, 1.0]  # Green

    if show_histogram:
        plt.figure(figsize=(8, 4))
        bins = np.arange(0, map.n_orientations + 2) - 0.5  # Center bins
        plt.hist(inconsistency_counts, bins=bins, edgecolor='black', color='darkred') # type: ignore
        plt.xticks(np.arange(0, map.n_orientations + 1))
        plt.xlabel("Number of Inconsistent Orientations per Frame")
        plt.ylabel("Number of Frames")
        plt.title("Histogram of FK/IK Inconsistencies")
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.tight_layout()
        plt.show()

    return colors