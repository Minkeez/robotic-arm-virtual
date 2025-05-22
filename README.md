# 6DOF Robotic Arm Virtual Simulator

A fully software-based robotic arm simulator that supports:

- Forward & Inverse Kinematics (FK/IK)
- Customizable 6 DOF configuration
- 3D visualization with animation
- Path planning (A\*, RRT - WIP)
- Clean, modular codebase (Python)

> Designed as a portfolio-ready robotics simulation and educational tools.
> No physical hardware required.

---

### Project Structure

```
robotic_arm_virtual/
├── config/ # Robot configuration (DOF, links, limits)
├── ik/ # Inverse and forward kinematics modules
├── simulation/ # Visualization and animation logic
├── planning/ # Path planning algorithms (coming soon)
├── input/ # Sample command files
├── utils/ # Geometry tools and shared math
├── tests/ # Unit tests for FK/IK
├── main.py # Entry point to run simulation
├── requirements.txt # Required Python packages
└── README.md # You're here
```

---

### Features

- [x] **6 DOF robot** with custom DH parameters
- [x] Real-time forward kinematics
- [x] JSON-based robot config and commands
- [x] 3D rendering of joint positions
- [ ] Inverse kinematics with orientation (coming soon)
- [ ] Path planning with obstacle avoidance (coming soon)
- [ ] GUI or speech input (future)

---

### How It Works

- Joint angles → DH matrix → FK chain → 3D pose
- Commands loaded from `input/commands.json`
- Robot structure loaded from `config/robot_config.json`
- Output: animated visualization of joint motion

---

### Setup

```bash
git clone https://github.com/Minkeez/robotic-arm-virtual.git
cd robotic-arm-virtual
pip install -r requirements.txt
```

then run:

```bash
python main.py
```

---

### Example

| Feature                  | Screenshot / Output     |
| ------------------------ | ----------------------- |
| FK Pose                  | [insert GIF/screenshot] |
| Path Animation           | [coming soon]           |
| Simulated Joint Movement | [coming soon]           |

---

### Roadmap

- [ ] Clean Inverse kinematics (elbow up/down)
- [ ] Obstacle-aware path planing (A\*, RRT)
- [ ] GUI interface (streamlit / PyQt)
- [ ] Optional Whisper-based speech commands

---

### License

MIT License - free to use and modity.

---

### Author

Built by Phumin "HourCode" Udomdach as a personal robotics software project after graduating from Robotics and Automation Systems Engineering.

> Inspired by real-world robotic systems and simulation tools like ROS, MoveIt, and Gazebo.
