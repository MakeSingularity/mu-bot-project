#!/bin/bash
# Unified ROS 2 Jazzy Installation Script
# Ubuntu 24.04 LTS (Noble) - Cross-platform installer

set -e  # Exit on any error

echo "🚀 ROS 2 Jazzy Installer (Ubuntu 24.04+ Required)"
echo "================================================"
echo ""

# Verify Ubuntu 24.04+ requirement
if ! grep -q "ubuntu" /etc/os-release; then
    echo "❌ This installer requires Ubuntu"
    echo "Please use Ubuntu 24.04 LTS or newer"
    exit 1
fi

DISTRO=$(lsb_release -cs)
VERSION=$(lsb_release -rs)

# Check Ubuntu version compatibility
case "$DISTRO" in
    "noble")
        echo "✅ Ubuntu 24.04 Noble - Perfect for ROS 2 Jazzy"
        ;;
    "oracular"|"plucky")
        echo "✅ Ubuntu $VERSION ($DISTRO) - Compatible with ROS 2 Jazzy"
        DISTRO="noble"  # Use Noble packages for newer versions
        ;;
    "jammy"|"focal")
        echo "❌ Ubuntu $VERSION ($DISTRO) is too old for ROS 2 Jazzy"
        echo "Please upgrade to Ubuntu 24.04 LTS"
        exit 1
        ;;
    *)
        echo "⚠️  Ubuntu $VERSION ($DISTRO) - Using Noble packages"
        DISTRO="noble"
        ;;
esac

echo "Using Ubuntu $DISTRO repository for ROS 2 Jazzy packages"# Install prerequisites
echo "📦 Installing prerequisites..."
sudo apt update
sudo apt install -y curl gnupg lsb-release apt-transport-https ca-certificates

# Clean up any existing ROS setup
echo "🧹 Cleaning up existing ROS configuration..."
sudo rm -f /etc/apt/sources.list.d/ros2*.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 GPG key and repository
echo "🔑 Adding ROS 2 Jazzy repository..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $DISTRO main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
echo "🔄 Updating package list..."
sudo apt update

echo "✅ ROS 2 Jazzy repository configured successfully!"
echo "You can now install ROS 2 Jazzy packages with apt install"
