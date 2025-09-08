#!/bin/bash
# G1 Simulation Setup

set -e

echo "🎭 Setting up G1 simulation..."

cd ~/g1_workspace

# Install packages
pip3 install --user mujoco numpy matplotlib

# Setup G1 model
cd unitree_ros/robots/g1_description
source /opt/ros/foxy/setup.bash 2>/dev/null || true

# Convert model if xacro available
if command -v xacro >/dev/null 2>&1; then
    xacro xacro/g1.urdf.xacro > urdf/g1.urdf 2>/dev/null || echo "Using fallback model"
fi

cd ~/g1_workspace

# Test
python3 -c "import mujoco; print('✅ Ready')" || {
    echo "❌ MuJoCo failed"
    exit 1
}

echo "✅ Setup complete!"
echo "🚀 Run: cd ~/g1_workspace/dance_examples && python3 dance_simulator.py"