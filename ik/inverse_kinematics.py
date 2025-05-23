import numpy as np
from math import atan2, sqrt, acos, degrees, radians

def inverse_kinematics(x, y, z, config):
  L1, L2, L3 = config['link_lengths'][:3]

  # Step 1: Solve theta1 (base rotation)
  theta1 = degrees(atan2(y, x))

  # Step 2: Calculate wrist position relative to base
  r = sqrt(x**2 + y**2)
  z_eff = z - L1 # subtract base height

  # Step 3: Solve theta2, theta3 using cosing law
  D = (r**2 + z_eff**2 - L2**2 - L3**2) / (2 * L2 * L3)
  if D < -1 or D > 1:
    raise ValueError("Target position is unreachable")
  
  theta3 = degrees(acos(D)) # Elbow angle
  theta2 = degrees(atan2(z_eff, r) - atan2(L3 * np.sin(radians(theta3)), L2 + L3 * np.cos(radians(theta3))))

  # Return angles for J1-J3
  return [theta1, theta2, theta3]