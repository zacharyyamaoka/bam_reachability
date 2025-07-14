#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

from bam_ros_utils.msgs.geometry_converter import get_pose
import numpy as np  
from bam_ros_utils.msgs.visualization_converter import get_mesh_marker
from bam_ros_utils.msgs.np_converter import to_numpy


def main(args=None):
    rclpy.init(args=args)

    node = Node("test_marker_node")

    marker_pub = node.create_publisher(Marker, "test_marker_publisher", 10)

    pose = get_pose([0,0,0],[0,0,0])

# 
    # https://docs.foxglove.dev/docs/visualization/panels/3d#ros-markers
    marker = get_mesh_marker(
                            mesh_path="file:///home/bam/python_ws/bam_reachability/convex_hull.stl",
                            # mesh_path="package://bam_descriptions/meshes/gripper_v1/hand.stl",
                            position = to_numpy(pose.position),
                            quaterion = to_numpy(pose.orientation),
                            frame_id = 'base_link',
                            time_stamp = node.get_clock().now().to_msg(),
                            color=[0.0, 0.0, 1.0, 1.0])


    print(marker)
    marker_pub.publish(marker)
    
    # Spin for ~1 second to ensure message delivery
    start = node.get_clock().now().seconds_nanoseconds()[0]
    while node.get_clock().now().seconds_nanoseconds()[0] - start < 5:
        rclpy.spin_once(node, timeout_sec=0.1)

    # rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    # marker = get_mesh_marker(mesh_path="package://bam_descriptions/meshes/six_dof/hand_aruco_marker.stl",
    #                         position = to_numpy(mesh_pose_stamped.pose.position),
    #                         quaterion = to_numpy(mesh_pose_stamped.pose.orientation),
    #                         frame_id = self.planning_frame,
    #                         time_stamp = self.get_clock().now().to_msg(),
    #                         color=[0.0, 1.0, 0.0, 0.2])

            # self.aruco_marker_pub[key].publish(get_mesh_marker(obj.mesh_path, pos, rot, frame_id, stamp, (1.0,0.0,0.0,0.2)))
