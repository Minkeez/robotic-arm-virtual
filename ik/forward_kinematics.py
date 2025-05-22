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

def forward_kinematics(joint_angles):
  config = load_robot_config()
  L = config["link_lengths"]

  theta1, theta2, theta3, theta4, theta5, theta6 = joint_angles # degrees

  # DH params (a, alpha, d, theta)
  T1 = dh_transform(0,  90, L[0], theta1)
  T2 = dh_transform(0,   0, L[1], theta2)
  T3 = dh_transform(0,   0, L[2], theta3)
  T4 = dh_transform(0,   0,    0, theta4)
  T5 = dh_transform(0, -90,    0, theta5)
  T6 = dh_transform(0,   0, L[4], theta6)

  T = T1 @ T2 @ T3 @ T4 @ T5 @ T6
  position = T[:3, 3]
  orientation = T[:3, :3]

  return position, orientation