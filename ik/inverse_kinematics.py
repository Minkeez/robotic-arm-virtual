import numpy as np
from numpy.linalg import inv
from math import atan2, sqrt, acos, degrees, radians

from ik.forward_kinematics import dh_transform

def check_joint_limits(joint_angles, config):
  joint_limits = config["joint_limits"]
  for i, angle in enumerate(joint_angles):
    min_limit, max_limit = joint_limits[i]
    if not (min_limit <= angle <= max_limit):
      return False
  return True

def inverse_kinematics(x, y, z, config, elbow_up=False):
  L1, L2, L3 = config['link_lengths'][:3]

  theta1 = degrees(atan2(y, x))
  r = sqrt(x**2 + y**2)
  z_eff = z - L1

  D = (r**2 + z_eff**2 - L2**2 - L3**2) / (2 * L2 * L3)
  if D < -1 or D > 1:
    print("Warning: IK target is unreachable")
    return None
  
  # Choose sign based on elbow direction
  sign = -1 if elbow_up else 1
  theta3 = degrees(sign * acos(D))

  # Adjust theta2 based on elbow config
  theta2 = degrees(
    atan2(z_eff, r) - 
    atan2(L3 * np.sin(radians(theta3)), 
          L2 + L3 * np.cos(radians(theta3)))
  )

  return [theta1, theta2, theta3]

def solve_wrist_orientation(R_target, joint_angles_1_3, config):
  L = config["link_lengths"]
  theta1, theta2, theta3 = joint_angles_1_3

  # Build T0_3 using theta1-theta3
  dh_params = [
    (0, 90, L[0], theta1),
    (0,  0, L[1], theta2),
    (0,  0, L[2], theta3)
  ]

  T0_3 = np.eye(4)
  for a, alpha, d, theta in dh_params:
    T0_3 = T0_3 @ dh_transform(a, alpha, d, theta)

  R0_3 = T0_3[:3, :3]
  R3_6 = inv(R0_3) @ R_target # desired wrist oreintation

  # Solve wrist joint angles from R3_6
  # For simplicity, assume:
  # R3_6 = Rz(theta4) * Ry(theta5) * Rz(theta6)
  # This is ZYZ Euler angles (common for wrists)

  theta5 = degrees(acos(R3_6[2, 2]))
  if np.sin(radians(theta5)) == 0:
    # Singularity: theta4 + theta6 can't be separated
    theta4 = 0
    theta6 = degrees(atan2(-R3_6[0, 1], R3_6[0, 0]))
  else:
    theta4 = degrees(atan2(R3_6[1, 2], R3_6[0, 2]))
    theta6 = degrees(atan2(R3_6[2, 1], -R3_6[2, 0]))

  return [theta4, theta5, theta6]

def inverse_kinematics_full(x, y, z, R_target, config, elbow_up=False):
  joint_angles_1_3 = inverse_kinematics(x, y, z, config, elbow_up=elbow_up)
  if joint_angles_1_3 is None:
    return None

  joint_angles_4_6 = solve_wrist_orientation(R_target, joint_angles_1_3, config)
  full_joint_angles = joint_angles_1_3 + joint_angles_4_6

  if not check_joint_limits(full_joint_angles, config):
    print("Warning: IK solution violates joint limits")
    return None

  return full_joint_angles