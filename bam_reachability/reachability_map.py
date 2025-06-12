#!/usr/bin/env python3

import numpy as np
from typing import Callable, Optional
import pickle
import matplotlib.pyplot as plt

"""
Make sure that orientation frames and IK tip are aligned

- generally IK tip is has +Z pointing out of the final link

"""
class ReachabilityMap():

    def __init__(self,
                 frames: np.ndarray,
                 orientations: np.ndarray,
                 IK, # function
                 FK=None, # function
                 ):

        assert frames.ndim == 2 and frames.shape[1] == 3  # shape: (N, 3)
        self.frames = frames 
        self.orientations = orientations
        self.map = []

        self.IK = IK
        self.FK = FK

        self.check_fk_ik_consitency = FK is not None

        self.calculate()

    def calculate(self):

        self.map = []
        n_frames = self.frames.shape[0]
        n_orientations = self.orientations.shape[0]
        total = n_frames * n_orientations
        count = 0

        for i in range(n_frames): 
            frame = self.frames[i,:] # shape (3,)
            info = {"ik_sols":[None] * n_orientations,
                    "success":[0] * n_orientations, # by default it's failed
                    "fk_sols":[None] * n_orientations,
                    "consistent":[1] * n_orientations # by default it's consistent
                    }

            for j in range(n_orientations):
                count += 1

                orientation = self.orientations[j,:] 

                pose = np.hstack((frame, orientation))  # shape (6,)

                ik_success, ik_sol = self.IK(pose)

                if ik_success:
                    info["ik_sols"][j] = ik_sol
                    info["success"][j] = 1
        
                    if self.check_fk_ik_consitency: # if IK wasn't succesful then you don't check this

                        fk_success, fk_sol = self.FK(ik_sol)

                        if not fk_success:
                            info["consistent"][j] = 0 # IK succeeded but FK didn't... this is an issue!
                            continue

                        info["fk_sols"][j] = fk_sol

                        if not np.allclose(fk_sol, pose):
                            print("[ERROR] IK/FK not consistent")
                            print("Pose: ", np.round(pose,6))
                            print("ik_sol: ", ik_success, np.round(ik_sol,6))
                            print("fk_sol: ", fk_success, np.round(fk_sol,6))
                            info["consistent"][j] = 0


                if count % 500 == 0 or count == total:
                    print(f"Processed {count} / {total} poses")

            info["success"] = np.array(info["success"])
            info["consistent"] = np.array(info["consistent"])
            
            self.map.append(info)

        assert len(self.map) == len(self.frames)

    def save(self, file_path: str):
        data = {
            "frames": self.frames,
            "orientations": self.orientations,
            "map": self.map,
        }
        with open(file_path, "wb") as f:
            pickle.dump(data, f)
        print(f"[Saved] Reachability map saved to {file_path}")

    @staticmethod
    def load(file_path: str, IK = None, FK = None):
        
        with open(file_path, "rb") as f:
            data = pickle.load(f)

        obj = ReachabilityMap.__new__(ReachabilityMap)
        obj.frames = data["frames"]
        obj.orientations = data["orientations"]
        obj.map = data["map"]
        obj.IK = IK
        obj.FK = FK
        obj.check_fk_ik_consitency = FK is not None
        print(f"[Loaded] Reachability map loaded from {file_path}")
        return obj
        
    def colorize(self, min_threshold=0.2, max_threshold=0.95, n_bins=5, histogram=True):
        """
        Return (frames, colors) where colors is (N, 4) RGBA array:
        - Blue = highly reachable
        - Red = poorly reachable
        - Alpha = 0 for unreachable frames

        If histogram=True, shows:
        - Raw score histogram
        - Binned score bar chart with readable bin labels
        """
        import matplotlib.pyplot as plt

        n_bins = n_bins - 2 # for the start and end ones
        scores = np.array([
            np.mean(info["success"]) for info in self.map
        ])  # (N,)
        print(scores)
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
            import matplotlib.pyplot as plt

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

        return self.frames, colors