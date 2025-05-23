import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot_arm(joint_positions):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  # Extract X, Y, Z
  xs = [p[0] for p in joint_positions]
  ys = [p[1] for p in joint_positions]
  zs = [p[2] for p in joint_positions]

  # Plot joints
  ax.plot(xs, ys, zs, '-o', linewidth=3, markersize=6, color="blue")

  # Set view limits
  ax.set_xlim([-50, 50])
  ax.set_ylim([-50, 50])
  ax.set_zlim([0, 60])

  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')

  ax.set_title("6DOF Robotic Arm Pose")
  plt.tight_layout()
  plt.show()