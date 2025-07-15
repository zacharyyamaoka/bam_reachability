import example_robot_data

robot = example_robot_data.load("panda")
model = robot.model
collision_model = robot.collision_model
visual_model = robot.visual_model

print(model)

for i, frame in enumerate(model.frames):
    print(f"[{i}] {frame.name} - {frame.type}")

global_joint_id = model.getJointId("panda_link0")
frame_id = model.getFrameId("panda_link7")
joint_id = model.frames[frame_id].parentJoint

print(global_joint_id)
print(frame_id)
print(joint_id)
print("-"*100)


robot = example_robot_data.load("ur5")
model = robot.model
collision_model = robot.collision_model
visual_model = robot.visual_model

print(model)

for i, frame in enumerate(model.frames):
    print(f"[{i}] {frame.name} - {frame.type}")

global_joint_id = model.getJointId("tool0")
frame_id = model.getFrameId("tool0")
joint_id = model.frames[frame_id].parentJoint

print(global_joint_id)
print(frame_id)
print(joint_id)
print("-"*100)







