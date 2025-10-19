#!/bin/bash
# Unified ROS 2 Jazzy Installation Script
# Cross-platform installer for Ubuntu and Debian systems

set -e  # Exit on any error

echo "🚀 ROS 2 Jazzy Cross-Platform Installer"
echo "======================================="
echo ""

# Detect OS and set appropriate repository
if grep -q "ubuntu" /etc/os-release; then
    echo "✅ Detected Ubuntu"
    DISTRO=$(lsb_release -cs)
    echo "Using Ubuntu $DISTRO repository"

    # Check if Ubuntu version supports Jazzy
    case "$DISTRO" in
        "noble"|"jammy")
            echo "✅ Ubuntu $DISTRO supports ROS 2 Jazzy"
            ;;
        *)
            echo "⚠️  Ubuntu $DISTRO may not have official Jazzy support, using Noble packages"
            DISTRO="noble"  # ROS 2 Jazzy is primarily built for Ubuntu 24.04 Noble
            ;;
    esac

elif grep -q "debian" /etc/os-release; then
    echo "✅ Detected Debian"
    DEBIAN_VERSION=$(grep VERSION_ID /etc/os-release | cut -d'"' -f2)
    echo "Debian version: $DEBIAN_VERSION"

    # Debian 13 (trixie) with Python 3.13 works well with Jazzy
    if [[ "$DEBIAN_VERSION" -ge "12" ]]; then
        DISTRO="noble"  # Use Ubuntu 24.04 Noble packages for modern Debian
        echo "Using Ubuntu Noble repository (compatible with modern Debian)"
    else
        DISTRO="jammy"  # Fallback for older Debian
        echo "Using Ubuntu Jammy repository (fallback for older Debian)"
    fi
else
    echo "⚠️  Unknown OS detected, defaulting to Ubuntu Noble"
    DISTRO="noble"
fi

# Install prerequisites
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
