# ROS 2 Dependencies for Emu Droid Project

This file lists the ROS 2 packages required for the emu droid project.
These must be installed via `apt` after installing ROS 2 Humble.

## Core ROS 2 Installation

```bash
# Install ROS 2 Humble (Ubuntu 22.04) - GPG Key Fix Included
sudo apt update && sudo apt install curl gnupg lsb-release

# IMPORTANT: Clean up any existing problematic ROS setup first
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Download ROS 2 GPG key to the correct location
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Verify the key was downloaded successfully
ls -la /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository with proper GPG key reference
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists (should work without GPG errors now)
sudo apt update

# If you still get GPG errors, use this alternative method:
# sudo apt install software-properties-common
# curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# sudo apt-add-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
# sudo apt update

# Install ROS 2 Humble desktop
sudo apt install ros-humble-desktop
```

## Required ROS 2 Packages

Install these packages after ROS 2 Humble installation:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Core ROS 2 packages (included in desktop install)
# - rclpy
# - std_msgs
# - sensor_msgs
# - geometry_msgs
# - cv_bridge
# - tf2_ros
# - tf2_geometry_msgs

# Additional packages for emu droid
sudo apt install \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-urdf-parser-py \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-msgs \
    ros-humble-audio-msgs \
    ros-humble-control-msgs \
    ros-humble-trajectory-msgs

# System dependencies for Python packages
sudo apt install \
    portaudio19-dev \
    python3-dev \
    build-essential \
    libasound2-dev \
    libffi-dev \
    libssl-dev
```

## Development Tools

```bash
# ROS 2 development tools
sudo apt install \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools

# Initialize rosdep
sudo rosdep init
rosdep update
```

## Verification

Test your ROS 2 installation:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Test basic functionality
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener

# Kill test nodes
pkill -f "demo_nodes"
```

## Auto-sourcing ROS 2

Add to your `~/.bashrc` for automatic sourcing:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/mu-bot/install/setup.bash" >> ~/.bashrc  # After building workspace
```

## Building the Emu Vision Package

```bash
# Navigate to workspace
cd ~/mu-bot

# Install dependencies for this workspace
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Troubleshooting ROS 2 Installation

### GPG Key Errors
If you see errors like "NO_PUBKEY F42ED6FBAB17C654" or "repository is not signed":

```bash
# Clean up existing setup
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Try the installation again with the updated method above
# Or use the legacy method:
sudo apt install software-properties-common
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-add-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update
sudo apt install ros-humble-desktop
```

### Network Issues
If downloads fail:
```bash
# Try different mirror
sudo sed -i 's|packages.ros.org|packages.osrfoundation.org|g' /etc/apt/sources.list.d/ros2.list
sudo apt update
```

### Permission Issues
```bash
# Ensure proper permissions
sudo chown root:root /usr/share/keyrings/ros-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg
```
