import numpy as np, json

from ik.inverse_kinematics import inverse_kinematics_full

with open("config/robot_config.json", "r") as f:
  config = json.load(f)

# Desired pose
target_pos = [30.0, 20.0, 25.0] # target XYZ in cm - Reachable
target_pos = [100, 100, 100] # test - Unreachable
target_rot = np.eye(3) # identity rotation for now (no rotation)

angles = inverse_kinematics_full(*target_pos, target_rot, config)
if angles is None:
  print("No valid IK solution found.")
else:
  print("theta1-theta6:", angles)