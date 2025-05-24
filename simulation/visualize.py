import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from ik.forward_kinematics import compute_all_joint_positions

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

def interpolate_joint_angles(start_angles, end_angles, steps):
  return [
    np.linspace(start, end, steps)
    for start, end in zip(start_angles, end_angles)
  ]

def animate_arm_motion(start_angles, end_angles, config, steps=100, interval=50):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  plt.tight_layout()

  # Setup plot limits
  ax.set_xlim([-50, 50])
  ax.set_ylim([-50, 50])
  ax.set_zlim([0, 60])
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_title("6DOF Robotic Arm Motion")

  # Interpolation
  angle_paths = interpolate_joint_angles(start_angles, end_angles, steps)

  line, = ax.plot([], [], [], '-o', color="blue", linewidth=3, markersize=6)

  def update(frame):
    # Interpolated joint angles at this frame
    current_angles = [angles[frame] for angles in angle_paths]
    positions = compute_all_joint_positions(current_angles, config)

    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]

    line.set_data(xs, ys)
    line.set_3d_properties(zs)
    return line,

  ani = FuncAnimation(fig, update, frames=steps, interval=interval, blit=False)
  plt.show()