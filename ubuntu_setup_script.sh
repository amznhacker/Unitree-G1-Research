#!/bin/bash
# Unitree G1 Complete Development Environment Setup
# For Ubuntu 20.04 LTS

set -e  # Exit on any error

echo "🤖 Starting Unitree G1 Development Environment Setup..."

# Update system
echo "📦 Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y

# Install required dependencies for Unitree SDK2
echo "🔧 Installing Unitree SDK2 dependencies..."
sudo apt-get install -y \
    cmake \
    g++ \
    build-essential \
    libyaml-cpp-dev \
    libeigen3-dev \
    libboost-all-dev \
    libspdlog-dev \
    libfmt-dev \
    git \
    wget \
    curl

# Install Python and pip
echo "🐍 Installing Python development tools..."
sudo apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv

# Install ROS2 Foxy (for Ubuntu 20.04)
echo "🤖 Installing ROS2 Foxy..."
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-foxy-desktop

# Source ROS2 in bashrc
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Install Python packages for simulation and dance
echo "🎭 Installing Python packages for simulation and dance..."
pip3 install --user \
    numpy \
    matplotlib \
    scipy \
    mujoco \
    gymnasium \
    pybullet \
    opencv-python \
    pygame \
    music21

# Create development workspace
echo "📁 Creating development workspace..."
mkdir -p ~/g1_workspace/src
cd ~/g1_workspace

# Clone Unitree SDK2
echo "📥 Cloning Unitree SDK2..."
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2

# Build Unitree SDK2
echo "🔨 Building Unitree SDK2..."
mkdir build
cd build
cmake ..
make -j$(nproc)

# Install SDK to system
echo "📦 Installing SDK to system..."
sudo make install

# Clone G1 URDF models
echo "📥 Cloning G1 robot models..."
cd ~/g1_workspace
git clone https://github.com/unitreerobotics/unitree_ros.git

# Create examples directory
mkdir -p ~/g1_workspace/dance_examples

# Set up environment variables
echo "🌍 Setting up environment variables..."
echo "export G1_WORKSPACE=~/g1_workspace" >> ~/.bashrc
echo "export PYTHONPATH=\$PYTHONPATH:~/g1_workspace/unitree_sdk2/python" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc

# Source the bashrc
source ~/.bashrc

echo "✅ Setup complete! Please run 'source ~/.bashrc' or restart your terminal."
echo "🎉 Your G1 development environment is ready!"
echo ""
echo "Next steps:"
echo "1. cd ~/g1_workspace/dance_examples"
echo "2. Run the dance examples we've created"
echo "3. Start developing your own movements!"