import numpy as np, json

from ik.inverse_kinematics import inverse_kinematics_full

with open("config/robot_config.json", "r") as f:
  config = json.load(f)

# Desired pose
target_pos = [30.0, 20.0, 25.0] # target XYZ in cm
target_rot = np.eye(3) # identity rotation for now (no rotation)

angles = inverse_kinematics_full(*target_pos, target_rot, config)
print("theta1-theta6:", angles)