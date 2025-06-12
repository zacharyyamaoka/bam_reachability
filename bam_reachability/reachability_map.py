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
                 FK, # function
                 ):

        assert frames.ndim == 2 and frames.shape[1] == 3  # shape: (N, 3)
        self.frames = frames 
        self.orientations = orientations

        self.n_frames = self.frames.shape[0]
        self.n_orientations = self.orientations.shape[0]
        self.total_count = self.n_frames * self.n_orientations


        self.ik_map = []
        self.fk_map = []

        self.IK = IK
        self.FK = FK
        
        self.calculate_ik_fk_map()


    def new_ik_info(self) -> dict:
        ik_info = {
            "ik_sols":[None] * self.n_orientations,
            "success":[0] * self.n_orientations, # by default it's failed
            }
        return ik_info

    def new_fk_info(self) -> dict:
        fk_info = {
            "fk_sols":[None] * self.n_orientations,
            "consistent":[1] * self.n_orientations, # by default it's consistent
            "success":[0] * self.n_orientations, # by default it's failed
            }
        return fk_info
    
    def new_result_info(self) -> dict:
        result_info = {
            "pose_success":[0] * self.n_orientations, 
            "ik_success":[0] * self.n_orientations, 
            "ik_sol_success":[0] * self.n_orientations, 
            "fk_success":[0] * self.n_orientations, 
            "fk_sol_success":[0] * self.n_orientations, 
            }
        return result_info
    
    def calculate_ik_fk_map(self):

        self.ik_map = []
        self.fk_map = []
        count = 0

        for i in range(self.n_frames): 
            frame = self.frames[i,:] # shape (3,)

            ik_info = self.new_ik_info()
            fk_info = self.new_fk_info()

            for j in range(self.n_orientations):
                count += 1

                orientation = self.orientations[j,:] 

                pose = np.hstack((frame, orientation)) # shape (6,)

                ik_success, ik_sol = self.IK(pose)

                if ik_success:
                    ik_info["ik_sols"][j] = ik_sol
                    ik_info["success"][j] = 1

                fk_sol, consistent, fk_success = self.check_fk_consistency(pose, ik_success, ik_sol)
                fk_info["fk_sols"][j] = fk_sol
                fk_info["consistent"][j] = consistent
                fk_info["success"][j] = fk_success

                if count % 500 == 0 or count == self.total_count:
                    print(f"Processed {count} / {self.total_count} poses")

            ik_info["success"] = np.array(ik_info["success"])
            fk_info["consistent"] = np.array(fk_info["consistent"])
            fk_info["success"] = np.array(fk_info["success"])

            self.ik_map.append(ik_info)
            self.fk_map.append(fk_info)

        assert len(self.ik_map) == len(self.frames)

    def check_fk_consistency(self, pose, ik_success, ik_sol): # return fk_sol, consitent, fk_success
        fk_sol = None
        consistent = 1
        fk_success = 0

        if not ik_success:
            return fk_sol, consistent, fk_success
                
        fk_success, fk_sol = self.FK(ik_sol)

        if not fk_success:
            consistent = 0
            return None, consistent, fk_success
      
        if not self.pose_is_close(fk_sol, pose):
            # print("[ERROR] IK/FK not consistent")
            # print("ik_sol: ", ik_success, np.round(ik_sol,6))
            consistent = 0

        return fk_sol, consistent, fk_success
    

    def calculate_fk_map(self):
        """ Not needed as done at same time as IK"""

        self.fk_map = []
        count = 0

        for i in range(self.n_frames): 
            frame = self.frames[i,:] # shape (3,)
            ik_info = self.ik_map[i]
            fk_info = self.new_fk_info()

            for j in range(self.n_orientations):
                count += 1

                ik_success = ik_info["success"][j]
                if ik_success:

                    orientation = self.orientations[j,:] 
                    pose = np.hstack((frame, orientation)) # shape (6,)
                    ik_sol = ik_info["ik_sols"][j] # stored solution for this pose

                    fk_sol, consistent, fk_success = self.check_fk_consistency(pose, ik_success, ik_sol)
                    fk_info["fk_sols"][j] = fk_sol
                    fk_info["consistent"][j] = consistent
                    fk_info["success"][j] = fk_success

            fk_info["consistent"] = np.array(fk_info["consistent"])
            fk_info["success"] = np.array(fk_info["success"])
            self.fk_map.append(fk_info)
    
    def compare_map(self, map_2):
        """
        Unpacks two maps and checks that the order and magnitude of values are the same!
        """

        result_map = []

        frames_1, orientations_1 = self.frames, self.orientations
        ik_map_1, fk_map_1 = self.ik_map, self.fk_map

        frames_2, orientations_2 = map_2.frames, map_2.orientations
        ik_map_2, fk_map_2 = map_2.ik_map, map_2.fk_map

        # First check that lengths are all equal
        # Not needed actually, as long as its correct doesn't matter if its longer
        count = 0
        for i in range(self.n_frames): 
            frame_1 = frames_1[i,:] # shape (3,)
            frame_2 = frames_2[i,:] # shape (3,)

            ik_info_1 = ik_map_1[i]
            ik_info_2 = ik_map_2[i]

            fk_info_1 = fk_map_1[i]
            fk_info_2 = fk_map_2[i]

            result_info = self.new_result_info()

            for j in range(self.n_orientations):
                count += 1

                # Check frame poses are the same. 
                # Important that inputs are the same, or outputs are not likely to be!
                orientation_1 = orientations_1[j,:] # shape (3,)
                orientation_2 = orientations_2[j,:] # shape (3,)

                pose_1 = np.hstack((frame_1, orientation_1)) # shape (6,)
                pose_2 = np.hstack((frame_2, orientation_2)) # shape (6,)

                pose_success = self.pose_is_close(pose_1, pose_2)
                result_info["pose_success"][j] = pose_success

                # Check IK results are the Same
                ik_success_1 = ik_info_1["success"][j] 
                ik_success_2 = ik_info_2["success"][j] 
                result_info["ik_success"][j] = (ik_success_1 == ik_success_2)

                ik_sol_1 = ik_info_1["ik_sols"][j] 
                ik_sol_2 = ik_info_2["ik_sols"][j] 

                ik_success = self.ik_sol_is_close(ik_sol_1, ik_sol_2)
                result_info["ik_sol_success"][j] = ik_success

                # Check FK results are the Same
                # This may fail if they don't have the same collision checking anymore!
                fk_success_1 = fk_info_1["success"][j] 
                fk_success_2 = fk_info_2["success"][j] 
                result_info["fk_success"][j] = (fk_success_1 == fk_success_2)

                fk_sol_1 = fk_info_1["fk_sols"][j] 
                fk_sol_2 = fk_info_2["fk_sols"][j] 

                fk_success = self.fk_sol_is_close(fk_sol_1, fk_sol_2)
                result_info["fk_sol_success"][j] = fk_success

            result_info["ik_success"] = np.array(result_info["ik_success"])
            result_info["ik_sol_success"] = np.array(result_info["ik_sol_success"])
            result_info["fk_success"] = np.array(result_info["fk_success"])
            result_info["fk_sol_success"] = np.array(result_info["fk_sol_success"])

            result_map.append(result_info)

        return result_map

    
    def check_for_none(self, sol_1, sol_2):
        """
        First bool is if you should continue computation, second bool is if they are the same:
        """
        if sol_1 is None and sol_2 is None:
            return True, True  # If both are None → considered equal → return True, True
        elif sol_1 is not None and sol_2 is not None:
            return False, None # If both are not None → proceed to actual comparison → return False, None
        else:
            return True, False # If one is None → considered unequal → return True, False

    def ik_sol_is_close(self, sol_1, sol_2):

        found_none, success = self.check_for_none(sol_1, sol_2)
        if found_none: return success

        success = np.allclose(sol_1, sol_2)
        if not success:
            print(f"[ERROR] IK solutions do not match")
            print(f"ik_sol_1: ", np.round(sol_1,6))
            print(f"ik_sol_2: ", np.round(sol_2,6))
        return success
    
    def fk_sol_is_close(self, sol_1, sol_2):
        found_none, success = self.check_for_none(sol_1, sol_2)
        if found_none: return success
    
        success = np.allclose(sol_1, sol_2)
        if not success:
            print(f"[ERROR] FK solutions do not match")
            print(f"fk_sol_1: ", np.round(sol_1,6))
            print(f"fk_sol_2: ", np.round(sol_2,6))
        return success
    
    def pose_is_close(self, pose_1, pose_2):
        """
        I thought I may need to do quaternion difference, but because it's small angles, you can just compare directly!
        """
        success = np.allclose(pose_1, pose_2)
        if not success:
            print(f"[ERROR] Target poses do not match")
            print(f"pose_1: ", np.round(pose_1,6))
            print(f"pose_2: ", np.round(pose_2,6))
        return success

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