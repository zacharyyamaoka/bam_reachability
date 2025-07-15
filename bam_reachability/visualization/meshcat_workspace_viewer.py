#!/usr/bin/env python3

# BAM
from pin_utils import MeshcatClient
from bam_reachability.reachability_map import ReachabilityMap
from bam_reachability.reachability_workspace import ReachabilityWorkspace

# PYTHON
import numpy as np
import pinocchio as pin
import time
import random
from xacrodoc import XacroDoc

# Option to display the full map, or select workspaces
# Option to display paritucalr points in workspace via calling the sample method...

class MeshcatWorkspaceViewer(MeshcatClient):

    @classmethod
    def from_xacro(cls, workspace: ReachabilityWorkspace, xacro_path: str, mesh_directory: str, zmq_url=""):

        doc = XacroDoc.from_file(xacro_path)
        with doc.temp_urdf_file_path() as urdf_path:
            model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_directory)

        return cls(workspace, model, collision_model, visual_model, zmq_url)

    @classmethod
    def from_urdf(cls, workspace: ReachabilityWorkspace, urdf_path: str, mesh_directory: str, zmq_url=""):
        model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_directory)
        return cls(workspace, model, collision_model, visual_model, zmq_url)


    def __init__(self, workspace: ReachabilityWorkspace, model, collision_model, visual_model, zmq_url=""):

        super().__init__(model, collision_model, visual_model, zmq_url=zmq_url)

        self.workspace = workspace

        self.model = model
        self.collision_model = collision_model
        self.visual_model = visual_model

        self.size = 0.05


        self.workspace_names = ["full", "reachable", "four_dof", "hemisphere", "dexterous"]
        self.workspace_points: list[np.ndarray] = [workspace.map.positions, workspace.reachable_points, workspace.four_dof.positions, workspace.hemisphere.positions, workspace.dexterous.positions]
        self.n_workspaces = len(self.workspace_names)

        self.workspace_index = 0
        self.pose = None

        self.display_point_cloud()
        self.display_pose()

    def rainbow_colors(self, num_points: int) -> np.ndarray:
        # Use HSV colormap to generate a rainbow gradient
        hsv = np.zeros((num_points, 3))
        hsv[:, 0] = np.linspace(0, 1, num_points)  # Hue from 0 to 1
        hsv[:, 1] = 1.0  # Full saturation
        hsv[:, 2] = 1.0  # Full value

        import matplotlib.colors as mcolors
        rgb = mcolors.hsv_to_rgb(hsv)  # shape (num_points, 3)
        # Add alpha channel (fully opaque)
        rgba = np.concatenate([rgb, np.ones((num_points, 1))], axis=1)  # (num_points, 4)
        colors = rgba.T  # (4, N)

        return colors

    def display_point_cloud(self):
        points = self.workspace_points[self.workspace_index].T  # (3, N)
        print("Displaying point cloud for workspace: ", self.workspace_names[self.workspace_index])

        # Add a rainbow gradient to the colors
        num_points = points.shape[1]
        colors = self.rainbow_colors(num_points)

        super().display_pointcloud(points, colors=colors, size=self.size)

     
    def display_pose(self):

        workspace_name = self.workspace_names[self.workspace_index]

        if workspace_name == "full" or workspace_name == "reachable":
            q = self.workspace.map.sample_valid_ik_sol()
        else:
            q = self.workspace.sample_q(workspace_name)

        super().display(q)
          

    def random_frame(self):
        self.display_pose()

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
        print("  [← / →]    → Next / Previous orientation")
        print("  [↑ / ↓]    → Previous / Next frame")
        print("  [+]        → Increase point size")
        print("  [-]        → Decrease point size")
        print("  [Q / Esc]  → Quit")

        def on_press(key):
            try:
                k = key.char
            except AttributeError:
                k = key.name

            if k == "space":
                self.display_pose()
            elif k == 'up':
                self.workspace_index = (self.workspace_index + 1) % self.n_workspaces
                self.display_pose()
                self.display_point_cloud()
            elif k == 'down':
                self.workspace_index = (self.workspace_index - 1) % self.n_workspaces
                self.display_pose()
                self.display_point_cloud()
            elif k == 'right':
                self.workspace_index = (self.workspace_index + 1) % self.n_workspaces
                self.display_pose()
                self.display_point_cloud()
            elif k == 'left':
                self.workspace_index = (self.workspace_index - 1) % self.n_workspaces
                self.display_pose()
                self.display_point_cloud()
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

    # UR is the most representative robot for this....
    import example_robot_data
    from bam_reachability.reachability_map import ReachabilityMap
    from bam_reachability.visualization.colorize_map import colorize_reachability, colorize_inconsistency
    from bam_reachability.kin_wrapper import UR5eOffsetWristKinWrapper

    map = ReachabilityMap.load("/home/bam/python_ws/bam_reachability/maps/ur/ur5e/ur5e_table_1.2x1.2x1.0_0.10_180x45x360_15_jul_2025.pkl")

    table_pose_matrix = np.eye(4)
    z_axis = -1 * table_pose_matrix[:3, 2]

    workspace = ReachabilityWorkspace(map, z_axis)

    K = UR5eOffsetWristKinWrapper()

    viewer = MeshcatWorkspaceViewer.from_xacro(workspace, K.xacro_path, K.mesh_directory)
    viewer.run()