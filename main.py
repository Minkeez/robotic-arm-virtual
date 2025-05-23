# from ik.forward_kinematics import forward_kinematics

# joint_angles = [0, 45, -30, 0, 90, 45] # degree
# pos, rot = forward_kinematics(joint_angles)

# print("End-Effector Position:", pos)
# print("Rotation Matrix:\n", rot)

from ik.inverse_kinematics import inverse_kinematics
import json

with open("config/robot_config.json", "r") as f:
  config = json.load(f)

target = [30.0, 20.0, 25.0] # target XYZ in cm
angles = inverse_kinematics(*target, config)

print("J1-J3 Angles:", angles)