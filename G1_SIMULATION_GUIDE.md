# G1 Simulation to Real Robot Guide

## 🎯 Current Setup

### What We Have
1. **MuJoCo Simulation** - Visual 3D robot dancing
2. **Dance Sequences** - Joint angle trajectories over time
3. **G1 Model** - 23DOF kinematic structure matching real robot

### What It Does
- Generates realistic dance movements
- Tests joint limits and timing
- Visualizes robot motion before deployment

## 🔄 Simulation to Real Robot Workflow

### 1. Simulation (Safe Testing)
```bash
cd ~/g1_workspace/dance_examples
python3 dance_simulator.py
```
**Output:** Joint trajectories tested in virtual environment

### 2. Real Robot Deployment
```python
from unitree_sdk2py import Robot
from cumbia_dance import CumbiaDance

# Generate dance in simulation
dance = CumbiaDance()
sequence = dance.generate_sequence(duration=20)

# Connect to real G1
robot = Robot()
robot.Init()

# Deploy to real robot
for frame in sequence:
    robot.SendJointCommand(frame['joints'])
    time.sleep(0.02)  # 50Hz control
```

## 🤖 Joint Mapping (Simulation ↔ Real Robot)

### G1 23DOF Structure
```
Simulation Joint Names    →    Real G1 Joint IDs
waist_yaw                →    Joint 0
left_hip_yaw             →    Joint 1
left_hip_roll            →    Joint 2
left_hip_pitch           →    Joint 3
left_knee                →    Joint 4
left_ankle_pitch         →    Joint 5
left_ankle_roll          →    Joint 6
right_hip_yaw            →    Joint 7
right_hip_roll           →    Joint 8
right_hip_pitch          →    Joint 9
right_knee               →    Joint 10
right_ankle_pitch        →    Joint 11
right_ankle_roll         →    Joint 12
left_shoulder_pitch      →    Joint 13
left_shoulder_roll       →    Joint 14
left_shoulder_yaw        →    Joint 15
left_elbow               →    Joint 16
left_wrist_roll          →    Joint 17
right_shoulder_pitch     →    Joint 18
right_shoulder_roll      →    Joint 19
right_shoulder_yaw       →    Joint 20
right_elbow              →    Joint 21
right_wrist_roll         →    Joint 22
```

## 🎭 How Dance Translation Works

### 1. Dance Classes Generate Trajectories
```python
# Each dance creates time-based joint commands
frame = {
    'time': 1.5,  # seconds
    'joints': [0.1, 0.2, -0.3, ...]  # 23 joint angles in radians
}
```

### 2. Simulation Tests Safety
- Joint limits: -π to +π (configurable)
- Collision detection
- Balance validation
- Timing verification

### 3. Real Robot Execution
```python
# Send joint commands at 50Hz
for frame in sequence:
    joint_commands = frame['joints']
    robot.SendJointCommand(joint_commands)
    time.sleep(0.02)
```

## 🚨 Safety Pipeline

### Simulation Validation
1. **Joint Limits** - Ensure angles within robot capabilities
2. **Smoothness** - No sudden movements that could damage robot
3. **Balance** - Center of mass stays within support polygon
4. **Timing** - Realistic movement speeds

### Real Robot Safeguards
1. **Start Small** - Test with reduced amplitude first
2. **Emergency Stop** - Always have kill switch ready
3. **Gradual Deployment** - Increase complexity slowly
4. **Monitor Battery** - Stop at 25% charge

## 🔧 Development Workflow

### Step 1: Create Dance in Simulation
```python
# Modify existing dance or create new one
class MyDance(BaseDance):
    def generate_sequence(self, duration=20):
        # Your movement logic here
        return sequence
```

### Step 2: Test in MuJoCo
```bash
python3 my_dance.py  # Visual verification
```

### Step 3: Deploy to Real Robot
```python
# Only after thorough simulation testing
robot.SendJointCommand(validated_sequence)
```

## 📊 What Gets Translated

### From Simulation
- **Joint angles** (radians)
- **Timing** (50Hz updates)
- **Sequence duration**
- **Movement smoothness**

### To Real Robot
- **Motor commands** (same joint angles)
- **Control frequency** (same 50Hz)
- **Safety limits** (enforced by robot firmware)

## 🎯 Key Points

1. **Simulation = Safe Testing Environment**
2. **Same joint structure** as real G1 (23DOF)
3. **Direct translation** of joint angles
4. **Real robot adds safety layers**
5. **Always test in simulation first**

## 🚀 Quick Start Example

```bash
# 1. Test dance in simulation
python3 cumbia_dance.py

# 2. If it looks good, deploy to real robot
python3 deploy_to_robot.py cumbia_dance
```

The simulation gives you confidence that your dance will work safely on the real G1 robot.