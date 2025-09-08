# Unitree G1 Development Setup Guide

## Development Environment Requirements

### Recommended System
- **OS**: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: NVIDIA GPU with CUDA support (for simulation acceleration)
- **Storage**: 50GB+ free space

### Alternative Options
- **Docker**: Use Ubuntu container on Windows/macOS
- **WSL2**: Windows Subsystem for Linux with Ubuntu
- **Virtual Machine**: VMware/VirtualBox with Ubuntu

## Core Development Stack

### 1. Unitree SDK2 (Required)
```bash
# Clone the official SDK
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2

# Follow installation instructions in repository
```

### 2. ROS2 (Recommended)
```bash
# Ubuntu 22.04 - ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full

# Ubuntu 20.04 - ROS2 Foxy
sudo apt install ros-foxy-desktop-full
```

### 3. MuJoCo Physics Engine (Yes, you need this)
```bash
# Install MuJoCo
pip install mujoco
# OR download from: https://github.com/google-deepmind/mujoco
```

**Why MuJoCo?**
- High-fidelity physics simulation
- Real-time performance
- Industry standard for robotics
- Required for advanced G1 simulations

## Quick Setup Script

### Ubuntu Installation
```bash
#!/bin/bash
# G1 Development Environment Setup

# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y git cmake build-essential python3-pip

# Install ROS2 (Humble for Ubuntu 22.04)
sudo apt install -y ros-humble-desktop-full
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install Python packages
pip3 install numpy matplotlib mujoco gymnasium

# Clone Unitree SDK2
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
# Follow build instructions

# Clone G1 URDF models
git clone https://github.com/unitreerobotics/unitree_ros.git
```

## G1 Model Files

### URDF Models (Required)
```bash
# Download G1 robot descriptions
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots/g1_description
```

**Available Models:**
- G1-EDU Standard (23 DOF)
- G1-EDU Advanced (29 DOF)
- G1-EDU Ultimate A-D (29 DOF + various hands)

### Simplified Models
- **V1.0**: Basic simulation models
- **V2.0**: Enhanced detail models

## Development Workflow

### 1. Simulation First Approach
```python
# Basic MuJoCo simulation setup
import mujoco
import mujoco.viewer

# Load G1 model
model = mujoco.MjModel.from_xml_path('path/to/g1.xml')
data = mujoco.MjData(model)

# Run simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 2. Custom Movement Development
```python
# Example: Custom walking gait
def custom_walk_cycle(model, data, step_time):
    # Define joint targets
    joint_targets = {
        'left_hip_pitch': 0.2,
        'right_hip_pitch': -0.2,
        # ... more joints
    }
    
    # Apply control
    for joint_name, target in joint_targets.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        data.ctrl[joint_id] = target
```

### 3. Real Robot Deployment
```python
# Using Unitree SDK2
from unitree_sdk2py import *

# Connect to robot
robot = Robot()
robot.Init()

# Send commands developed in simulation
robot.SendCommand(your_movement_data)
```

## Development Tools

### Simulation Environments
1. **MuJoCo** - Physics simulation
2. **Gazebo** - ROS integration
3. **PyBullet** - Python-friendly alternative
4. **Isaac Sim** - NVIDIA's advanced simulator

### Motion Planning
1. **MoveIt2** - ROS2 motion planning
2. **OMPL** - Open Motion Planning Library
3. **Pinocchio** - Fast dynamics computations

### Visualization
1. **RViz2** - ROS visualization
2. **MuJoCo Viewer** - Built-in viewer
3. **Matplotlib** - Data plotting

## Getting Started Steps

### Phase 1: Setup (Week 1)
1. Install Ubuntu 22.04 LTS
2. Set up development environment
3. Clone and build Unitree SDK2
4. Download G1 URDF models
5. Test basic MuJoCo simulation

### Phase 2: Learning (Week 2-3)
1. Study G1 kinematics and dynamics
2. Run existing simulation examples
3. Modify basic movements
4. Learn Unitree SDK2 API

### Phase 3: Development (Week 4+)
1. Create custom movement patterns
2. Test in simulation extensively
3. Validate with real robot (carefully!)
4. Iterate and improve

## Safety Considerations

### Simulation Testing
- Test ALL movements in simulation first
- Validate joint limits and constraints
- Check for collisions and stability
- Verify battery consumption estimates

### Real Robot Testing
- Start with minimal movements
- Use emergency stop at all times
- Follow all safety protocols from operating procedures
- Have backup/recovery plans

## Resources and Documentation

### Official Documentation
- [Unitree SDK2 GitHub](https://github.com/unitreerobotics/unitree_sdk2)
- [G1 SDK Development Guide](https://support.unitree.com/)
- [URDF Models](https://github.com/unitreerobotics/unitree_ros)

### Learning Resources
- MuJoCo Documentation
- ROS2 Tutorials
- Robotics simulation courses
- Unitree community forums

### Example Projects
```bash
# Clone example projects
git clone https://github.com/unitreerobotics/unitree_sdk2_examples.git
```

## Troubleshooting

### Common Issues
1. **CUDA not found**: Install NVIDIA drivers and CUDA toolkit
2. **ROS2 build errors**: Check dependencies and environment
3. **MuJoCo license**: Use free version for academic/personal use
4. **G1 connection**: Verify network and SDK configuration

### Performance Optimization
- Use GPU acceleration for simulation
- Optimize control loop frequency
- Profile code for bottlenecks
- Use efficient data structures

---

**Next Steps**: Start with Phase 1 setup, then move to basic simulation examples before attempting custom movements.

**Remember**: Always test in simulation before deploying to real robot!