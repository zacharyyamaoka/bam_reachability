#!/usr/bin/env python3
   
import numpy as np
import matplotlib.pyplot as plt
from bam_reachability.reachability_map import ReachabilityMap

def colorize_map(map: ReachabilityMap, min_threshold=0.2, max_threshold=0.95, n_bins=5, histogram=True):
    """
    Return (frames, colors) where colors is (N, 4) RGBA array:
    - Blue = highly reachable
    - Red = poorly reachable
    - Alpha = 0 for unreachable frames

    If histogram=True, shows:
    - Raw score histogram
    - Binned score bar chart with readable bin labels
    """

    n_bins = n_bins - 2 # for the start and end ones
    scores = np.array([np.mean(info["success"]) for info in map.ik_map])  # (N,)
    colors = np.zeros((len(scores), 4))  # (R, G, B, A)
    reachable_mask = scores > 0.0
    colors[~reachable_mask, 3] = 0.0  # Alpha = 0 for unreachable points

    binned_scores = np.zeros_like(scores)

    if np.any(reachable_mask):
        # Step 1: Define bin edges, starting from 0.0
        bin_step = (max_threshold - min_threshold) / n_bins
        bin_edges = [0.0]  # start from 0
        for i in range(n_bins+1):
            bin_edges.append(min_threshold + i * bin_step)
        bin_edges.append(1.0)  # always end at 1.0
        print("Bin Edges: ", bin_edges)

        # Step 2: Assign bin ID to each reachable score
        bin_ids = np.full(len(scores), -1, dtype=int)  # -1 = unreachable

        for idx, score in enumerate(scores):
            if not reachable_mask[idx]:
                continue
            for i, edge in enumerate(bin_edges[1:]):
                if score <= edge:
                    bin_ids[idx] = i
                    break

        # Step 3: Normalize bin ID to [0, 1] range for colormap
        bin_id_float = bin_ids.astype(float)
        bin_id_float[bin_id_float == -1] = 0  # ignore unreachable for color mapping
        norm_vals = np.clip(bin_id_float / max(n_bins, 1), 0.0, 1.0)

        cmap = plt.get_cmap("coolwarm_r")
        rgba = cmap(norm_vals[reachable_mask])
        colors[reachable_mask, :3] = rgba[:, :3]
        colors[reachable_mask, 3] = 1.0

        binned_scores = norm_vals

    if histogram:
        
        fig, axes = plt.subplots(1, 2, figsize=(12, 4))

        # It may look a bit funny as it spans [0,1), so first bin starts at 0, last bin includes up to 1.0
        
        # Raw histogram
        raw_counts, raw_bins, _ = axes[0].hist(scores, bins=20, color='gray', edgecolor='black')
        axes[0].set_title("Raw Reachability Scores")
        axes[0].set_xlabel("Score")
        axes[0].set_ylabel("Count")

        # Add bin edge ticks
        axes[0].set_xticks(raw_bins)
        axes[0].set_xticklabels([f"{edge:.2f}" for edge in raw_bins], rotation=45, ha='right')

        # Add counts above bars
        for x, count in zip(raw_bins[:-1], raw_counts):
            if count > 0:
                axes[0].text(x + (raw_bins[1] - raw_bins[0]) / 2, count, str(int(count)),
                            ha='center', va='bottom', fontsize=8)

        # Binned bar chart
        if n_bins > 1:
            hist_counts, _ = np.histogram(scores[reachable_mask], bins=bin_edges)
            print("Bin Counts: ", hist_counts)

            bar_positions = np.arange(len(hist_counts))
            bar_labels = [f"{bin_edges[i]:.2f} – {bin_edges[i+1]:.2f}" for i in range(len(hist_counts))]

            bars = axes[1].bar(bar_positions, hist_counts, width=0.8, color='steelblue', edgecolor='black')
            axes[1].bar(bar_positions, hist_counts, width=0.8, color='steelblue', edgecolor='black')
            axes[1].set_xticks(bar_positions)
            axes[1].set_xticklabels(bar_labels, rotation=45, ha='right')
            axes[1].set_title("Binned Reachability Scores")
            axes[1].set_xlabel("Score Bin")
            axes[1].set_ylabel("Count")

                            # Add counts above bars
            for bar in bars:
                height = bar.get_height()
            
                axes[1].text(bar.get_x() + bar.get_width() / 2, height, str(int(height)),
                            ha='center', va='bottom', fontsize=8)
        else:
            axes[1].text(0.5, 0.5, "Binning disabled (n_bins=1)", ha='center', va='center')
            axes[1].set_axis_off()

        plt.tight_layout()
        plt.show()

    return map.frames, colors