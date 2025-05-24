import numpy as np, json

from ik.forward_kinematics import compute_all_joint_positions
from ik.inverse_kinematics import inverse_kinematics_full
from simulation.visualize import animate_arm_motion

with open("config/robot_config.json", "r") as f:
  config = json.load(f)

# # Desired pose
# target_pos = [30, 20, 25] # target XYZ in cm - Reachable
# target_rot = np.eye(3) # identity rotation for now (no rotation)

# angles_up = inverse_kinematics_full(*target_pos, target_rot, config, elbow_up=True)
# angles_down = inverse_kinematics_full(*target_pos, target_rot, config, elbow_up=False)

# if angles_up is not None:
#   print("Elbow-Up Solution:", [round(a, 2) for a in angles_up])
# else:
#   print("Elbow-Up: No solution")

# if angles_down is not None:
#   print("Elbow-Down Solution:", [round(a, 2) for a in angles_down])
# else:
#   print("Elbow-Down: No solution")

start = [0, 0, 0, 0, 0, 0]
target = [0, 45, -30, 0, 90, 45]

animate_arm_motion(start, target, config)