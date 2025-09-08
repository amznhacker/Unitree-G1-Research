#!/bin/bash
# G1 Development Environment Setup

set -e

echo "🤖 Setting up G1 environment..."

# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install dependencies
sudo apt-get install -y cmake g++ build-essential git wget curl python3 python3-pip

# Install ROS2 Foxy
sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update && sudo apt install -y ros-foxy-desktop ros-foxy-xacro

# Setup workspace
mkdir -p ~/g1_workspace
cd ~/g1_workspace

# Clone repositories
git clone https://github.com/unitreerobotics/unitree_sdk2.git
git clone https://github.com/unitreerobotics/unitree_ros.git

# Build SDK
cd unitree_sdk2
mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install

# Environment
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "export G1_WORKSPACE=~/g1_workspace" >> ~/.bashrc

echo "✅ Setup complete!"
echo "🚀 Next: ./setup_simulation.sh"