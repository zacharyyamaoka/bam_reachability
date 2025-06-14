import numpy as np
from bam_kinematics_dynamics import MeshcatClient
from bam_reachability.reachability_map import ReachabilityMap
import pinocchio as pin
import time
import random

class MeshcatMapViewer:
    def __init__(self, reach_map: ReachabilityMap, colors: np.ndarray, model, collision_model, visual_model, size=0.05, meshcat_url=None):
        """
        Args:
            reach_map: ReachabilityMap object
            colors: (N, 3) or (N, 4) numpy array for coloring each frame point
            model, collision_model, visual_model: Pinocchio robot models
        """
        self.reach_map = reach_map
        self.colors = colors
        self.model = model
        self.collision_model = collision_model
        self.visual_model = visual_model

        self.meshcat_client = MeshcatClient(model, collision_model, visual_model, True)
        self.size = size
    
        self.frame_index = 0
        self.orient_index = 0
        self.pose = None

        self.inconsistent_mode = False

        self.display_point_cloud()
        self.display_current_pose()

    def display_point_cloud(self):
        points = self.reach_map.frames.T  # (3, N)
        if self.colors.shape[1] == 4:
            color = self.colors[:, :3].T
        else:
            color = self.colors.T

        self.meshcat_client.display_pointcloud(points, color, size=self.size)

        # fk_info = {
        #     "fk_sols":[None] * self.n_orientations,
        #     "consistent":[1] * self.n_orientations, # by default it's consistent
        #     "success":[0] * self.n_orientations, # by default it's failed
        #     }
        
    def display_current_pose(self):
        ik_info = self.reach_map.ik_map[self.frame_index]
        fk_info = self.reach_map.fk_map[self.frame_index]

        ik_success_list = ik_info["success"]
        fk_success_list = fk_info["success"]
        fk_consistent_list = fk_info["consistent"]
            
        if ik_success_list[self.orient_index]:
            q = ik_info["ik_sols"][self.orient_index]
            self.meshcat_client.display(q)
            print(f"[✓] Displaying frame {self.frame_index}, orientation {self.orient_index}")
            found = True
        else:
            # Try to find next successful orientation
            found = False
            # for i in range(len(ik_success_list)):
            #     next_idx = (self.orient_index + i) % len(ik_success_list)
            #     if ik_success_list[next_idx]:
            #         self.orient_index = next_idx
            #         q = ik_info["ik_sols"][self.orient_index]
            #         self.meshcat_client.display(q)
            #         print(f"[✓] Found fallback orientation {self.orient_index} for frame {self.frame_index}")
            #         found = True
            #         break

            if not found:
                self.meshcat_client.display(np.array([0,0,0,0,0,0]))
                print(f"[✗] No valid IK solutions for frame {self.frame_index}")



        # Show Target pose always
        if self.reach_map.per_frame_orientations:
            orientation = self.reach_map.orientations[self.frame_index, self.orient_index, :]
        else:
            orientation = self.reach_map.orientations[self.orient_index, :]
        position = self.reach_map.frames[self.frame_index]
        pose = np.hstack((position, orientation))
        self.meshcat_client.display_xyzrpy(pose[:3], pose[3:], name="target_pose")

        if found:
            if fk_success_list[self.orient_index]:
                fk_pose = fk_info["fk_sols"][self.orient_index]  # shape (6,) = [xyz rpy]
                self.meshcat_client.display_xyzrpy(fk_pose[:3], fk_pose[3:], name="fk_pose")
        else:
            print(f"[✗] No valid FK solutions for frame {self.frame_index}")
            self.meshcat_client.clear_xyzrpy(name="fk_pose")

        # do this after you have found the orient index
        def mark_index(lst):
            return [
                f"[{int(v)}]" if i == self.orient_index else f" {int(v)} "
                for i, v in enumerate(lst)
            ]
        print(f"Frame {self.frame_index} ik_success list: {' '.join(mark_index(ik_success_list))}")
        print(f"Frame {self.frame_index} fk_success list: {' '.join(mark_index(fk_success_list))}")
        print(f"Frame {self.frame_index} consistent list: {' '.join(mark_index(fk_consistent_list))}")

    def next_orientation(self):
        n = len(self.reach_map.ik_map[self.frame_index]["success"])
        self.orient_index = (self.orient_index + 1) % n
        self.display_current_pose()
     

    def prev_orientation(self):
        n = len(self.reach_map.ik_map[self.frame_index]["success"])
        self.orient_index = (self.orient_index - 1) % n
        self.display_current_pose()

    def random_frame(self):
        self.frame_index = random.randint(0, self.reach_map.n_frames - 1)
        # self.orient_index = 0
        self.display_current_pose()

    def increase_point_size(self, step=0.01):
        self.size += step
        print(f"Point size increased to {self.size:.3f}")
        self.display_point_cloud()

    def decrease_point_size(self, step=0.01):
        self.size = max(0.001, self.size - step)
        print(f"Point size decreased to {self.size:.3f}")
        self.display_point_cloud()

    def jump_to_nearest_inconsistent(self, forward=True):
        start = self.frame_index
        n = self.reach_map.n_frames

        offset_range = range(1, n) if forward else range(-1, -n, -1)

        for offset in offset_range:
            idx = (start + offset) % n
            fk_info = self.reach_map.fk_map[idx]
            if np.any(~np.array(fk_info["consistent"]).astype(bool)):
                self.frame_index = idx
                # self.orient_index = 0
                print(f"[!] Jumped to inconsistent frame {idx}")
                self.display_current_pose()
                return

        print("[✓] No inconsistent frames found.")

    def run(self):
        from pynput import keyboard

        print("Viewer running... Press keys to interact.")
        print("  [Space]    → Random frame")
        print("  [← / →]    → Next / Previous orientation")
        print("  [↑ / ↓]    → Previous / Next frame")
        print("  [+]        → Increase point size")
        print("  [-]        → Decrease point size")
        print("  [C]        → Toggle inconsistent mode (↑/↓ jump to nearest bad frame)")
        print("  [Q / Esc]  → Quit")

        def on_press(key):
            try:
                k = key.char
            except AttributeError:
                k = key.name
            print("Key pressed: ", k)
            if k == "space":
                self.random_frame()
            elif k == 'up':
                if self.inconsistent_mode:
                    self.jump_to_nearest_inconsistent(forward=True)
                else:
                    self.frame_index = (self.frame_index + 1) % self.reach_map.n_frames
                    # self.orient_index = 0
                    self.display_current_pose()
            elif k == 'down':
                if self.inconsistent_mode:
                    self.jump_to_nearest_inconsistent(forward=False)
                else:
                    self.frame_index = (self.frame_index - 1) % self.reach_map.n_frames
                    # self.orient_index = 0
                    self.display_current_pose()
            elif k == 'right':
                self.next_orientation()
            elif k == 'left':
                self.prev_orientation()
            elif k in ['+', '=']:
                self.increase_point_size()
            elif k in ['-', '_']:
                self.decrease_point_size()

            elif k in ['c', 'C']:
                self.inconsistent_mode = not self.inconsistent_mode
                print(f"Inconsistent mode {'ON' if self.inconsistent_mode else 'OFF'}")
                
            elif k in ['q', 'Q', 'esc']:
                print("Exiting viewer...")
                return False  # Stop listener

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

if __name__ == "__main__":
    from bam_kinematics_dynamics import urdf_to_models

    from bam_reachability.reachability_map import ReachabilityMap
    from bam_reachability.visualizer import colorize_map, colorize_inconsistency
    from bam_descriptions import get_robot_params
    import pinocchio as pin
    # map_path = "/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur_moveit_map_13_jun_2025.pkl"
    map_path = "/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur_offset_wrist_map_13_jun_2025.pkl"
    map_path = "/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur_inconsistent_offset_wrist_map_13_jun_2025.pkl"
    map_path="/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur_inconsistent_offset_wrist_map_14_jun_2025.pkl"
    map_path="/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur_table_offset_wrist_map_14_jun_2025.pkl"
    # Load reachability map
    rmap = ReachabilityMap.load(map_path)

    # Get colorized point cloud
    frames, colors = colorize_map(rmap, histogram=True)
    # points, colors = colorize_inconsistency(rmap)


    rp = get_robot_params('ur')
 
    model, collision_model, visual_model = pin.buildModelsFromUrdf(rp.urdf_path, rp.mesh_package_name)

    viewer = MeshcatMapViewer(rmap, colors, model, collision_model, visual_model)
    viewer.run()