from ik.forward_kinematics import forward_kinematics

joint_angles = [0, 45, -30, 0, 90, 45] # degree
pos, rot = forward_kinematics(joint_angles)

print("End-Effector Position:", pos)
print("Rotation Matrix:\n", rot)