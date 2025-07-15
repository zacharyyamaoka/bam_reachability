#!/usr/bin/env python3

# BAM
from bam_reachability.visualization.meshcat_client import MeshcatClient
from bam_reachability.reachability_map import ReachabilityMap
from bam_descriptions import RobotParam

# PYTHON
import numpy as np
import pinocchio as pin
import time
import random

class MeshcatMapViewer:

    @classmethod
    def from_urdf(cls, map: ReachabilityMap, colors: np.ndarray, urdf_path: str, mesh_package_name: str, size=0.05, meshcat_url=""):

        model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_package_name)

        return cls(map, colors, model, collision_model, visual_model, size, meshcat_url)


    def __init__(self, reach_map: ReachabilityMap, colors: np.ndarray, model, collision_model, visual_model, size=0.05, meshcat_url=""):
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

        self.meshcat_client = MeshcatClient(model, collision_model, visual_model, meshcat_url)
        self.size = size
    
        self.pos_index = 0
        self.orient_index = 0
        self.pose = None

        self.inconsistent_mode = False

        self.display_point_cloud()
        self.display_current_pose()

    def display_point_cloud(self):
        points = self.reach_map.positions.T  # (3, N)

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
        ik_info = self.reach_map.ik_map[self.pos_index]
        fk_info = self.reach_map.fk_map[self.pos_index]
            
        if ik_info.success[self.orient_index]:
            q = ik_info.ik_sols[self.orient_index]
            self.meshcat_client.display(q)
            print(f"[✓] Displaying frame {self.pos_index}, orientation {self.orient_index}")
            found = True
        else:
            # Try to find next successful orientation
            found = False
            # for i in range(len(ik_info.success)):
            #     next_idx = (self.orient_index + i) % len(ik_info.success)
            #     if ik_info.success[next_idx]:
            #         self.orient_index = next_idx
            #         q = ik_info["ik_sols"][self.orient_index]
            #         self.meshcat_client.display(q)
            #         print(f"[✓] Found fallback orientation {self.orient_index} for frame {self.pos_index}")
            #         found = True
            #         break

            if not found:
                self.meshcat_client.display(np.array([0,0,0,0,0,0]))
                print(f"[✗] No valid IK solutions for frame {self.pos_index}")


        target_pose_matrix = self.reach_map.get_pose_matrix(self.pos_index, self.orient_index)
        self.meshcat_client.display_pose_matrix(target_pose_matrix, name="target_pose")

        if found:
            if fk_info.success[self.orient_index]:
                fk_pose = fk_info.fk_sols[self.orient_index]  # shape (6,) = [xyz rpy]
                self.meshcat_client.display_pose_matrix(fk_pose, name="fk_pose") # type: ignore
        else:
            print(f"[✗] No valid FK solutions for frame {self.pos_index}")
            self.meshcat_client.clear_pose_matrix(name="fk_pose")

        # do this after you have found the orient index
        def mark_index(lst):
            return [
                f"[{int(v)}]" if i == self.orient_index else f" {int(v)} "
                for i, v in enumerate(lst)
            ]
        print(f"Frame {self.pos_index} ik_success list: {' '.join(mark_index(ik_info.success))}")
        print(f"Frame {self.pos_index} fk_success list: {' '.join(mark_index(fk_info.success))}")
        print(f"Frame {self.pos_index} consistent list: {' '.join(mark_index(fk_info.consistent))}")

    def next_orientation(self):
        n = len(self.reach_map.ik_map[self.pos_index].success)
        self.orient_index = (self.orient_index + 1) % n
        self.display_current_pose()
     

    def prev_orientation(self):
        n = len(self.reach_map.ik_map[self.pos_index].success)
        self.orient_index = (self.orient_index - 1) % n
        self.display_current_pose()

    def random_frame(self):
        self.pos_index = random.randint(0, self.reach_map.n_positions - 1)
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
        start = self.pos_index
        n = self.reach_map.n_positions

        offset_range = range(1, n) if forward else range(-1, -n, -1)

        for offset in offset_range:
            idx = (start + offset) % n
            fk_info = self.reach_map.fk_map[idx]
            if np.any(~np.array(fk_info.consistent).astype(bool)):
                self.pos_index = idx
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
                    self.pos_index = (self.pos_index + 1) % self.reach_map.n_positions
                    # self.orient_index = 0
                    self.display_current_pose()
            elif k == 'down':
                if self.inconsistent_mode:
                    self.jump_to_nearest_inconsistent(forward=False)
                else:
                    self.pos_index = (self.pos_index - 1) % self.reach_map.n_positions
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

    # UR is the most representative robot for this....
    import example_robot_data
    from bam_reachability.reachability_map import ReachabilityMap
    from bam_reachability.visualization.colorize_map import colorize_reachability, colorize_inconsistency

    robot = example_robot_data.load("ur10")
    model = robot.model
    collision_model = robot.collision_model
    visual_model = robot.visual_model


    file_path ="/home/bam/python_ws/bam_reachability/maps/ur/ur10_table_1.2x1.2x1.0_0.20_90x45x360_map_14_jul_2025.pkl"
    map = ReachabilityMap.load(file_path)

    colors = colorize_reachability(map, show_histogram=True)
    # colors = colorize_inconsistency(map)


    viewer = MeshcatMapViewer(map, colors, model, collision_model, visual_model)
    viewer.run()