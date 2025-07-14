#!/usr/bin/env python3


# PYTHON
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g
import meshcat.transformations as tf

import os
import time
import numpy as np
import uuid


class MeshcatClient():


    @classmethod
    def from_urdf(cls, urdf_path: str, package_path: str, zmq_url="", color=None):
        model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, package_path)
        return cls(model, collision_model, visual_model, zmq_url=zmq_url, color=color)

    def __init__(self, model, collision_model, visual_model, zmq_url="", color=None):
        """
        Args:
            model: pinocchio model
            collision_model: pinocchio collision model
            visual_model: pinocchio visual model
            zmq_url: example: tcp://127.0.0.1:6000
            color: color of the robot
        """


        print("[UNCONFIGURED] MeshcatClient")

        self.model = model
        self.nq = model.nq
        self.collision_model = collision_model
        self.visual_model = visual_model

        self.viz = MeshcatVisualizer(model, collision_model, visual_model)
        self.update_frames = False
        self.color = color

        if zmq_url == "":
            self.viz.initViewer(open=True)
        else:
            print(f"Attemping to connect to Meshcat on: {zmq_url}") 
            self.viz.initViewer(zmq_url=zmq_url) # cli: meshcat-server
            print(f"Connected on: {zmq_url}")

        self.robot_prefix = None
        self.load_robot()

        # time.sleep(1.0) #wait for robot to load

        print("[READY] MeshcatClient")

    def load_robot(self):
        """(Re)load the robot model into the Meshcat viewer."""
        if self.model is None:
            raise ValueError("Model is not set. Cannot load robot.")
        
        random_id = np.random.randint(1000, 9999)
        self.robot_prefix = f"robot_{random_id}"
        self.viz.loadViewerModel(rootNodeName=self.robot_prefix, visual_color=self.color)

        q = pin.neutral(self.model)
        self.viz.display(q)
        print(f"[LOADED] Robot with prefix: {self.robot_prefix}")

    @property
    def zmq_url(self) -> str:
        """Return the ZMQ URL of the connected Meshcat viewer."""
        return self.viz.viewer.window.zmq_url
    
    def show_frame(self, frame_name, verbose=True):

        if verbose:
            print(f"\nFrames (n = {self.model.nframes}):")
            for i, frame in enumerate(self.model.frames):
                print(f"  Frame {i}: {frame.name}, type: {frame.type}, parent: {frame.parentJoint}")

        frame_id = self.model.getFrameId(frame_name)
        print(f"Showing Frame {frame_name} - {frame_id}")
        self.viz.displayFrames(True, [frame_id], axis_length=0.2, axis_width=3)
        self.viz.updateFrames()

    def reset(self):
        self.viz.reset()
        
    def clear_robot(self):
        """Remove the robot model from the Meshcat viewer."""
        if self.robot_prefix:
            self.viz.viewer[self.robot_prefix].delete()
            print(f"[CLEARED] Robot model: {self.robot_prefix}")
            self.robot_prefix = None

    def display(self, q: np.ndarray):

        if self.robot_prefix is None:
            print("[INFO] Robot not loaded — loading now.")
            self.load_robot()

        assert(q.shape[0] == self.nq)

        self.viz.display(q)

            
    def display_pointcloud(self, points: np.ndarray, colors: np.ndarray = None, size: float = 0.01, name: str = "point_cloud"):
        """
        Display a colored point cloud in Meshcat.

        Args:
            points: (3, N) numpy array of XYZ coordinates in world frame
            colors: (3, N) numpy array of RGB values in [0, 1]; defaults to green/red gradient on Z
            size: float point size in meters
            name: Meshcat path name for the point cloud
        """
        if points.shape[0] != 3 and points.shape[1] == 3:
            points = points.T

        assert points.shape[0] == 3, f"Expected points shape (3, N), got {points.shape}"

        if colors is None:
            num_points = points.shape[1]
            # Default to semi-transparent green: RGBA = [0, 1, 0, 0.5]
            rgba = np.array([[0.0], [1.0], [0.0], [0.5]])  # shape (4, 1)
            colors = np.repeat(rgba, num_points, axis=1)

        self.viz.viewer[name].set_object(
            g.PointCloud(position=points, color=colors, size=size)
        )
        self.viz.viewer[name].set_transform(tf.translation_matrix([0, 0, 0]))

    def clear_pointcloud(self, name: str = "point_cloud"):
        self.viz.viewer[name].delete()

    def display_pose_matrix(self, pose_matrix: np.ndarray, name="frame"):
        self.viz.viewer[name].set_object(g.triad(scale=0.2))
        self.viz.viewer[name].set_transform(pose_matrix)

    def clear_pose_matrix(self, name="frame"):
        self.viz.viewer[name].delete()

    def display_xyzrpy(self, xyz, rpy, name="frame"):

        # Convert to 4x4 transform matrix
        T = tf.euler_matrix(*rpy)
        T[:3, 3] = xyz

        # Display coordinate frame (Triad) at this pose
        self.viz.viewer[name].set_object(g.triad(scale=0.2))
        self.viz.viewer[name].set_transform(T)

    def clear_xyzrpy(self, name="frame"):
        """
        Remove a coordinate frame previously displayed with display_xyzrpy.
        
        Args:
            name: The Meshcat path name used when displaying the frame.
        """
        try:
            self.viz.viewer[name].delete()
            print(f"[CLEARED] Coordinate frame '{name}' removed.")
        except:
            print(f"[WARNING] Coordinate frame '{name}' not found in viewer.")

def main():

    import example_robot_data

    robot = example_robot_data.load("panda")
    model = robot.model
    collision_model = robot.collision_model
    visual_model = robot.visual_model

    # meshcat_client = MeshcatClient(model, collision_model, visual_model, "tcp://127.0.0.1:6000")
    meshcat_client = MeshcatClient(model, collision_model, visual_model)

    meshcat_client.display(np.zeros(model.nq))

    time.sleep(1)

    # traj = create_fake_traj(model)

    # points = 0.5 * np.random.rand(3, 1000) + np.array([[0.2], [0.2], [-0.5]])
    # meshcat_client.display_pointcloud(points)  # Uses default semi-transparent green
    # time.sleep(1)
    # points = 0.5 * np.random.rand(3, 1000) + np.array([[-0.2], [-0.2], [0.5]])
    # meshcat_client.display_pointcloud(points)  # Uses default semi-transparent green


if __name__ == '__main__':
    main()
