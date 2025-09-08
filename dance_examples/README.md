# G1 Dance Examples

Collection of dance sequences for the Unitree G1 humanoid robot.

## Available Dances

### 🎵 Cumbia Dance (`cumbia_dance.py`)
- **Tempo**: 120 BPM
- **Features**: Hip sway, flowing arm movements, figure-8 motion
- **Duration**: 30 seconds
- **Style**: Traditional Colombian cumbia with characteristic hip movements

### 💃 Salsa Dance (`salsa_dance.py`)
- **Tempo**: 180 BPM (fast)
- **Features**: Quick steps, dramatic spins, sharp arm movements
- **Duration**: 25 seconds
- **Style**: High-energy Latin dance with spins at 8s and 15s

### 💕 Bachata Dance (`bachata_dance.py`)
- **Tempo**: 130 BPM
- **Features**: Hip pops on beat 4, romantic arm gestures, side steps
- **Duration**: 30 seconds
- **Style**: Sensual Dominican dance with signature hip pops

### 🤖 Robot Dance (`robot_dance.py`)
- **Tempo**: 100 BPM (mechanical)
- **Features**: 90-degree angles, sharp movements, mechanical poses
- **Duration**: 40 seconds
- **Style**: Classic robot dance with precise, angular movements

## Usage

### Run Individual Dances
```bash
cd ~/g1_workspace/dance_examples
python3 cumbia_dance.py
python3 salsa_dance.py
python3 bachata_dance.py
python3 robot_dance.py
```

### Run Full Simulation
```bash
python3 dance_simulator.py
```

### Integration with Real G1
```python
from cumbia_dance import CumbiaDance
from unitree_sdk2py import *

# Generate dance sequence
dance = CumbiaDance()
sequence = dance.generate_sequence(duration=20)

# Connect to real robot
robot = Robot()
robot.Init()

# Execute dance
for frame in sequence:
    robot.SendJointCommand(frame['joints'])
    time.sleep(0.02)
```

## Dance Characteristics

| Dance | BPM | Key Features | Difficulty |
|-------|-----|--------------|------------|
| Cumbia | 120 | Hip sway, flowing arms | Easy |
| Salsa | 180 | Fast steps, spins | Hard |
| Bachata | 130 | Hip pops, romantic | Medium |
| Robot | 100 | Mechanical, precise | Easy |

## Safety Notes

⚠️ **Always test in simulation first**
⚠️ **Check joint limits before deployment**
⚠️ **Ensure stable footing for spins**
⚠️ **Monitor battery during long sequences**

## Customization

### Modify Tempo
```python
dance = CumbiaDance()
dance.bpm = 140  # Faster cumbia
sequence = dance.generate_sequence()
```

### Add Your Own Moves
```python
def custom_move_pattern(self, t):
    # Your custom movement logic
    return joint_angles

# Add to any dance class
```

### Create New Dances
Use existing dances as templates:
1. Copy a similar dance file
2. Modify tempo and movement patterns
3. Test in simulation
4. Deploy to robot

## Technical Details

- **Control Frequency**: 50Hz (0.02s intervals)
- **Joint Range**: ±π radians (adjust as needed)
- **Coordinate System**: Standard robotics conventions
- **File Format**: Python classes with numpy arrays

## Troubleshooting

**Dance too fast/slow**: Adjust `self.bpm` in dance class
**Jerky movements**: Increase smoothing in movement functions
**Joint limits exceeded**: Reduce amplitude values
**Simulation crashes**: Check MuJoCo installation

---

**Have fun dancing with your G1! 🕺💃**