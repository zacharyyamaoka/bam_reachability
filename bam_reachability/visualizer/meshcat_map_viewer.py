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

        self.display_point_cloud()
        self.display_current_pose()

    def display_point_cloud(self):
        points = self.reach_map.frames.T  # (3, N)
        if self.colors.shape[1] == 4:
            color = self.colors[:, :3].T
        else:
            color = self.colors.T

        self.meshcat_client.display_pointcloud(points, color, size=self.size)

    def display_current_pose(self):
        ik_info = self.reach_map.ik_map[self.frame_index]
        if ik_info["success"][self.orient_index]:
            q = ik_info["ik_sols"][self.orient_index]
            self.meshcat_client.display(q)

            print(f"[✓] Displaying frame {self.frame_index}, orientation {self.orient_index}")
        else:
            print(f"[✗] Frame {self.frame_index}, orientation {self.orient_index} has no IK solution")

    def next_orientation(self):
        self.orient_index = (self.orient_index + 1) % self.reach_map.n_orientations
        self.display_current_pose()

    def prev_orientation(self):
        self.orient_index = (self.orient_index - 1) % self.reach_map.n_orientations
        self.display_current_pose()

    def random_frame(self):
        self.frame_index = random.randint(0, self.reach_map.n_frames - 1)
        self.orient_index = 0
        self.display_current_pose()

    def increase_point_size(self, step=0.01):
        self.size += step
        print(f"Point size increased to {self.size:.3f}")
        self.display_point_cloud()

    def decrease_point_size(self, step=0.01):
        self.size = max(0.001, self.size - step)
        print(f"Point size decreased to {self.size:.3f}")
        self.display_point_cloud()
    def run(self):
        from pynput import keyboard

        print("Viewer running... Press keys to interact.")
        print("  [Space]    → Random frame")
        print("  [← / →]    → Previous / Next frame")
        print("  [↑ / ↓]    → Next / Previous orientation")
        print("  [+ / =]    → Increase point size")
        print("  [- / _]    → Decrease point size")
        print("  [Q / Esc]  → Quit")

        def on_press(key):
            try:
                k = key.char
            except AttributeError:
                k = key.name

            if k == ' ':
                self.random_frame()
            elif k == 'right':
                self.frame_index = (self.frame_index + 1) % self.reach_map.n_frames
                self.orient_index = 0
                self.display_current_pose()
            elif k == 'left':
                self.frame_index = (self.frame_index - 1) % self.reach_map.n_frames
                self.orient_index = 0
                self.display_current_pose()
            elif k == 'up':
                self.next_orientation()
            elif k == 'down':
                self.prev_orientation()
            elif k in ['+', '=']:
                self.increase_point_size()
            elif k in ['-', '_']:
                self.decrease_point_size()
            elif k in ['q', 'Q', 'esc']:
                print("Exiting viewer...")
                return False  # Stop listener

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

if __name__ == "__main__":
    from bam_kinematics_dynamics import urdf_to_models

    from bam_reachability.reachability_map import ReachabilityMap
    from bam_reachability.visualizer import colorize_map
    from bam_descriptions import get_robot_params
    import pinocchio as pin

    # Load reachability map
    rmap = ReachabilityMap.load("/home/bam/bam_ws/src/bam_plugins/bam_reachability/bam_reachability/analysis/ur/ur_moveit_map_13_jun_2025.pkl")

    # Get colorized point cloud
    frames, colors = colorize_map(rmap, histogram=False)


    rp = get_robot_params('ur')
 
    model, collision_model, visual_model = pin.buildModelsFromUrdf(rp.urdf_path, rp.mesh_package_name)

    viewer = MeshcatMapViewer(rmap, colors, model, collision_model, visual_model)
    viewer.run()