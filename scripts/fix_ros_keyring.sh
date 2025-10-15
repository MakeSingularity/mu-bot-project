#!/bin/bash
# ROS 2 GPG Keyring Fix Script
# Fixes the common "NO_PUBKEY F42ED6FBAB17C654" error when installing ROS 2

set -e  # Exit on any error

echo "🔧 ROS 2 GPG Keyring Fix Script"
echo "==============================="
echo ""

# Check if running on Ubuntu 22.04 (Jammy)
if ! grep -q "jammy" /etc/os-release; then
    echo "⚠️  Warning: This script is designed for Ubuntu 22.04 (Jammy)"
    echo "Your system may be different. Continue? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Exiting..."
        exit 1
    fi
fi

echo "📦 Installing required packages..."
sudo apt update && sudo apt install curl gnupg lsb-release

echo ""
echo "🧹 Cleaning up existing problematic ROS setup..."
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

echo ""
echo "🔑 Downloading ROS 2 GPG key..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Verify the key was downloaded
if [[ -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    echo "✅ GPG key downloaded successfully:"
    ls -la /usr/share/keyrings/ros-archive-keyring.gpg
else
    echo "❌ Failed to download GPG key"
    exit 1
fi

echo ""
echo "📝 Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo ""
echo "🔄 Updating package lists..."
if sudo apt update; then
    echo "✅ Package lists updated successfully!"
    echo ""
    echo "🚀 ROS 2 repository is now ready. You can install ROS 2 with:"
    echo "   sudo apt install ros-humble-desktop"
else
    echo "❌ Package update failed. Trying alternative method..."
    echo ""
    echo "🔄 Using legacy apt-key method..."

    # Remove the failed setup
    sudo rm -f /etc/apt/sources.list.d/ros2.list

    # Try legacy method
    sudo apt install software-properties-common
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-add-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"

    if sudo apt update; then
        echo "✅ Alternative method worked!"
        echo ""
        echo "🚀 ROS 2 repository is now ready. You can install ROS 2 with:"
        echo "   sudo apt install ros-humble-desktop"
    else
        echo "❌ Both methods failed. Please check your internet connection"
        echo "and try again, or report this issue on GitHub."
        exit 1
    fi
fi

echo ""
echo "🎉 ROS 2 GPG keyring fix completed successfully!"
echo ""
echo "Next steps:"
echo "1. Install ROS 2: sudo apt install ros-humble-desktop"
echo "2. Install system dependencies for Python packages:"
echo "   sudo apt install portaudio19-dev python3-dev build-essential libasound2-dev libffi-dev libssl-dev pkg-config"
echo "3. Follow the rest of the installation guide in README.md"
