from ik.inverse_kinematics import inverse_kinematics_full
import numpy as np

def solve_ik_path(path_cm, config, R_target=np.eye(3), elbow_up=False, z_height=10.0):
  ik_path = []
  for (x, y) in path_cm:
    ik_angles = inverse_kinematics_full(x, y, z_height, R_target, config, elbow_up=elbow_up)
    if ik_angles:
      ik_path.append(ik_angles)
    else:
      print(f"Warning: No IK solution at ({x},{y})")
  
  return ik_path