#!/usr/bin/env python3

import numpy as np
from typing import Callable, Optional
import pickle
import datetime  # Add this at the top if not already

from bam_reachability.utils import pose_is_close

"""
Make sure that orientation frames and IK tip are aligned

- generally IK tip is has +Z pointing out of the final link

#TODO refactor to rename frames -> positions which is more accurate!
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
        self.n_frames = self.frames.shape[0]

        if orientations.ndim == 3:
            assert orientations.shape[0] == frames.shape[0], "Mismatch in number of frames"
            self.per_frame_orientations = True
            self.n_orientations = orientations.shape[1]
        elif orientations.ndim == 2:
            self.per_frame_orientations = False
            self.n_orientations = orientations.shape[0]
        else:
            raise ValueError("Orientations must be of shape (M, 3) or (N, M, 3)")

        self.orientations = orientations


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

                if self.per_frame_orientations:
                    orientation = self.orientations[i, j, :]
                else:
                    orientation = self.orientations[j, :]

                pose = np.hstack((frame, orientation)) # shape (6,)
                ik_success, ik_sol = self.IK(pose)

                if ik_success:
                    ik_info["ik_sols"][j] = ik_sol
                    ik_info["success"][j] = 1

                fk_sol, consistent, fk_success = self.check_fk_consistency(pose, ik_success, ik_sol)
                fk_info["fk_sols"][j] = fk_sol
                fk_info["consistent"][j] = consistent
                fk_info["success"][j] = fk_success

                if count % 1000 == 0 or count == self.total_count:
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

        if not ik_success: # the ik_sol is ill-defined here so no point in checking!
            return fk_sol, consistent, fk_success
                
        fk_success, fk_sol = self.FK(ik_sol)

        if not fk_success:
            consistent = 0
            return None, consistent, fk_success
      
        if not pose_is_close(pose, fk_sol):
            print("IK/FK not consistent")
            print("ik_sol: ", ik_success, np.round(ik_sol,6))
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
    
    def save(self, file_path: str, timestamp=True):

        if timestamp:
            date_str = datetime.datetime.now().strftime("%d_%b_%Y").lower()  # e.g., '03_feb_2025'
            if file_path.endswith(".pkl"):
                file_path = file_path[:-4] + f"_{date_str}.pkl"
            else:
                file_path = file_path + f"_{date_str}.pkl"

        data = {
            "frames": self.frames,
            "orientations": self.orientations,
            "per_frame_orientations": self.per_frame_orientations,  # <-- NEW
            "ik_map": self.ik_map,
            "fk_map": self.fk_map,
        }
        with open(file_path, "wb") as f:
            pickle.dump(data, f)
        print(f"[Saved] Reachability map saved to {file_path}")

    @staticmethod
    def load(file_path: str, IK: Optional[Callable] = None, FK: Optional[Callable] = None):
        with open(file_path, "rb") as f:
            data = pickle.load(f)

        obj = ReachabilityMap.__new__(ReachabilityMap)
        obj.frames = data["frames"]
        obj.orientations = data["orientations"]
        obj.per_frame_orientations = data.get("per_frame_orientations", False)  # <-- NEW (fallback to False)
        obj.ik_map = data["ik_map"]
        obj.fk_map = data["fk_map"]

        obj.n_frames = obj.frames.shape[0]
        obj.n_orientations = (obj.orientations.shape[1] if obj.per_frame_orientations else obj.orientations.shape[0])
        obj.total_count = obj.n_frames * obj.n_orientations

        obj.IK = IK
        obj.FK = FK

        print(f"[Loaded] Reachability map loaded from {file_path}")
        return obj
        
    def reduce_random(self, num_frames: int, seed: Optional[int] = None):
        """
        Randomly sample a subset of frames to reduce the reachability map.

        Args:
            num_frames (int): Number of frames to keep.
            seed (int, optional): Random seed for reproducibility.
        """
        if num_frames >= self.n_frames:
            print(f"[Skip] Requested {num_frames} frames, but map has only {self.n_frames}. No reduction done.")
            return

        if seed is not None:
            np.random.seed(seed)

        selected_indices = np.random.choice(self.n_frames, size=num_frames, replace=False)
        selected_indices.sort()  # Optional: keep original order

        self.reduce(selected_indices)

    def reduce(self, frame_indices: np.ndarray):
        """
        Reduce the reachability map to only include frames at the given indices.

        Args:
            frame_indices (np.ndarray): Array of indices to keep (subset of original frame indices)
        """
        self.frames = self.frames[frame_indices]
        self.ik_map = [self.ik_map[i] for i in frame_indices]
        self.fk_map = [self.fk_map[i] for i in frame_indices]

        self.n_frames = self.frames.shape[0]
        self.total_count = self.n_frames * self.n_orientations

        print(f"[Reduced] Reachability map reduced to {self.n_frames} frames.")