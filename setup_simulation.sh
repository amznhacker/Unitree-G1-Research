#!/bin/bash
# G1 Simulation Setup - Run after ubuntu_setup_script.sh

set -e

echo "🎭 Setting up G1 simulation..."

# Navigate to workspace
cd ~/g1_workspace

# Install simulation packages
echo "🐍 Installing packages..."
pip3 install --user mujoco numpy matplotlib

# Test installation
echo "🧪 Testing..."
python3 -c "import mujoco; print('✅ Ready')" || {
    echo "❌ Failed - check MuJoCo installation"
    exit 1
}

echo "✅ Setup complete!"
echo "🚀 Run: cd ~/g1_workspace/dance_examples && python3 dance_simulator.py"