# Unitree G1 Robot Operations

Complete documentation and simulation environment for Unitree G1 humanoid robot.

## 🚀 Quick Start

```bash
# 1. Make scripts executable
chmod +x *.sh

# 2. Setup environment (Ubuntu only)
./ubuntu_setup_script.sh && source ~/.bashrc

# 3. Setup simulation
./setup_simulation.sh

# 4. Run dances
cd ~/g1_workspace/dance_examples
python3 dance_simulator.py
```

## 🎭 Available Dances

| Dance | Style | Duration |
|-------|-------|----------|
| **Cumbia** | Hip sway, flowing arms | 20s |
| **Salsa** | Fast steps with spins | 15s |
| **Bachata** | Hip pops, romantic | 20s |
| **Robot** | Mechanical precision | 25s |

```bash
# Run individual dances
python3 cumbia_dance.py
python3 salsa_dance.py
python3 bachata_dance.py
python3 robot_dance.py
```

## 🤖 Simulation to Real Robot

**How it works:** Simulation tests dance safety → Deploy to real G1

```bash
# 1. Test in simulation
cd ~/g1_workspace/dance_examples
python3 cumbia_dance.py

# 2. Deploy to real robot (if simulation looks good)
python3 deploy_to_robot.py cumbia 15
```

**Read:** [G1_SIMULATION_GUIDE.md](G1_SIMULATION_GUIDE.md) - Complete workflow explanation

## 📋 Documentation

### Field Operations
- **[G1_Operating_Procedures.md](G1_Operating_Procedures.md)** - Complete operating manual
- **[Pre_Operation_Checklist.md](Pre_Operation_Checklist.md)** - Field checklist
- **[Emergency_Procedures_Quick_Reference.md](Emergency_Procedures_Quick_Reference.md)** - Emergency guide

### Development
- **[G1_Development_Setup_Guide.md](G1_Development_Setup_Guide.md)** - Advanced development setup
- **[dance_examples/](dance_examples/)** - Python dance implementations

### Tracking
- **[Daily_Operation_Log_Template.md](Daily_Operation_Log_Template.md)** - Operation logging
- **[Maintenance_Parts_Tracking.md](Maintenance_Parts_Tracking.md)** - Equipment tracking

## 🚨 Safety Rules

1. **25% Battery Rule** - Return to base at 25% battery
2. **2-Hour Maximum** - Never exceed 2 hours operation
3. **Visual Contact** - Maintain constant visual contact
4. **Emergency Stop** - Always have emergency stop ready

---

**Emergency Help**: [Emergency_Procedures_Quick_Reference.md](Emergency_Procedures_Quick_Reference.md)