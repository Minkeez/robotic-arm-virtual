import numpy as np
import json
import os

def load_robot_config():
  config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'robot_config.json')
  with open(config_path, 'r') as f:
    return json.load(f)
  
def dh_transform(a, alpha, d, theta_deg):
  theta = np.radians(theta_deg)
  alpha = np.radians(alpha)
  return np.array([
    [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
    [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
    [0,              np.sin(alpha),                np.cos(alpha),               d],
    [0,              0,                            0,                           1]
  ])

def get_dh_table(joint_angles, link_lengths):
  # theta = input angle (joint_angles), all others fixed
  # Format: (a, alpha, d, theta)
  return [
    (0,  90, link_lengths[0], joint_angles[0]), # J1
    (0,   0, link_lengths[1], joint_angles[1]), # J2
    (0,   0, link_lengths[2], joint_angles[2]), # J3
    (0,  90,             0.0, joint_angles[3]), # J4
    (0, -90,             0.0, joint_angles[4]), # J5
    (0,   0, link_lengths[4], joint_angles[5])  # J6
  ]

def forward_kinematics(joint_angles):
  config = load_robot_config()
  dof = config['dof']
  link_lengths = config["link_lengths"]

  if len(joint_angles) != dof:
    raise ValueError(f"Expected {dof} joint angles, got {len(joint_angles)}.")
  
  dh_params = get_dh_table(joint_angles, link_lengths)

  T = np.eye(4)
  for a, alpha, d, theta in dh_params:
    T = T @ dh_transform(a, alpha, d, theta)

  position = T[:3, 3]
  orientation = T[:3, :3]

  return position, orientation

def compute_all_joint_positions(joint_angles, config):
  L = config["link_lengths"]
  theta1, theta2, theta3, theta4, theta5, theta6 = joint_angles

  dh_params = [
    (0,  90, L[0], theta1),
    (0,   0, L[1], theta2),
    (0,   0, L[2], theta3),
    (0,  90,  0.0, theta4),
    (0, -90,  0.0, theta5),
    (0,   0, L[4], theta6),
  ]

  T = np.eye(4)
  positions = [T[:3, 3].copy()] # Base position at origin

  for a, alpha, d, theta in dh_params:
    T = T @ dh_transform(a, alpha, d, theta)
    positions.append(T[:3, 3].copy())
  
  return positions # List of XYZ coords of joints 0-6