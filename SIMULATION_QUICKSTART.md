# G1 Simulation Quick Start

## ⚡ Setup (10 minutes)

```bash
# 1. Environment setup
./ubuntu_setup_script.sh && source ~/.bashrc

# 2. Simulation setup  
./setup_simulation.sh

# 3. Run simulation
cd ~/g1_workspace/dance_examples
python3 dance_simulator.py
```

## 🎮 Using Simulator

**Menu:** Select dance 1-5
- Cumbia (20s) - Hip sway
- Salsa (15s) - Fast steps  
- Bachata (20s) - Hip pops
- Robot (25s) - Mechanical
- Showcase - All dances

**Controls:** Close viewer window or Ctrl+C to exit

## 🔧 Individual Dances

```bash
python3 cumbia_dance.py
python3 salsa_dance.py
python3 bachata_dance.py
python3 robot_dance.py
```

## ❌ Troubleshooting

**MuJoCo error:** `pip3 install --user mujoco`
**Permission error:** `chmod +x *.sh`
**Import error:** `source ~/.bashrc`

## 🎉 Success!

MuJoCo viewer opens with dancing robot → Ready to develop!