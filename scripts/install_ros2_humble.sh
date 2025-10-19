#!/bin/bash
# Unified ROS 2 Humble Installation Script
# Cross-platform installer for Ubuntu and Debian systems

set -e  # Exit on any error

echo "🤖 ROS 2 Humble Cross-Platform Installer"
echo "========================================"
echo ""

# Detect OS and set appropriate repository
if grep -q "ubuntu" /etc/os-release; then
    echo "✅ Detected Ubuntu"
    DISTRO=$(lsb_release -cs)
    echo "Using Ubuntu $DISTRO repository"
elif grep -q "debian" /etc/os-release; then
    echo "✅ Detected Debian"
    DISTRO="jammy"  # ROS 2 uses Ubuntu repos for Debian compatibility
    echo "Using Ubuntu Jammy repository (standard for Debian)"
else
    echo "⚠️  Unknown OS detected, defaulting to Ubuntu Jammy"
    DISTRO="jammy"
fi

# Install prerequisites
echo "📦 Installing prerequisites..."
sudo apt update
sudo apt install -y curl gnupg lsb-release apt-transport-https ca-certificates

# Clean up any existing ROS setup
echo "🧹 Cleaning up existing ROS configuration..."
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 GPG key and repository
echo "🔑 Adding ROS 2 repository..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $DISTRO main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
echo "🔄 Updating package list..."
sudo apt update

echo "✅ ROS 2 repository configured successfully!"
echo "You can now install ROS 2 packages with apt install"
