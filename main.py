import json
import numpy as np

from ik.inverse_kinematics import inverse_kinematics_full
from simulation.visualize import animate_arm_motion

# Load config
with open("config/robot_config.json", "r") as f:
  config = json.load(f)

# Load command (target pose)
with open("input/commands.json", "r") as f:
  data = json.load(f)

target_pos = data["target"]["position"]
target_rot = np.array(data["target"]["orientation"])

# Use IK to get target joint angles
target_angles = inverse_kinematics_full(*target_pos, target_rot, config, elbow_up=False)

if target_angles is None:
  print("No valid IK solution found.")
else:
  print("Target joint angles:", [round(a, 2) for a in target_angles])

  # Set current joint state (FK pose)
  start_angles = [0, 0, 0, 0, 0, 0]

  # Animate from FK → IK target
  animate_arm_motion(start_angles, target_angles, config)