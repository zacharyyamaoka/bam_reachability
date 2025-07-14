from example_robot_data.robots_loader import PandaLoader, UR5Loader, EXAMPLE_ROBOT_DATA_SOURCE_DIR, EXAMPLE_ROBOT_DATA_MODEL_DIR
import example_robot_data
import os

# Print the base directory where all robot data is stored
print("EXAMPLE_ROBOT_DATA_SOURCE_DIR:", EXAMPLE_ROBOT_DATA_SOURCE_DIR)
print("EXAMPLE_ROBOT_DATA_MODEL_DIR:", EXAMPLE_ROBOT_DATA_MODEL_DIR)

# # For solo12:
# robot_urdf_path = os.path.join(ROBOT_DATA_PATH, "solo_description/robots/solo12.urdf")
# mesh_dir = os.path.join(ROBOT_DATA_PATH, "solo_description/meshes")

# print("URDF:", robot_urdf_path)
# print("Meshes:", mesh_dir)

panda_urdf_path = PandaLoader.urdf_filename
# panda_mesh_dir = PandaLoader.mesh_dir

print("URDF:", panda_urdf_path)
# print("Meshes:", panda_mesh_dir)

ur5_urdf_path = UR5Loader.urdf_filename
# ur5_mesh_dir = UR5Loader.mesh_dir

print("URDF:", ur5_urdf_path)

print(PandaLoader.model_path)

print(robot.visual_model)
print(robot.collision_model)
print(robot.model)
