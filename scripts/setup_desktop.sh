#!/bin/bash
# Desktop Development Station Setup Script
# Automated setup for primary development environment

set -e  # Exit on any error

echo "🖥️  Emu Droid Desktop Development Station Setup"
echo "=============================================="
echo ""

# Check if running on Ubuntu 22.04
if ! grep -q "jammy" /etc/os-release; then
    echo "⚠️  Warning: This script is designed for Ubuntu 22.04 (Jammy)"
    echo "Your system may be different. Continue? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Exiting..."
        exit 1
    fi
fi

echo "📦 Step 1: Installing ROS 2 Jazzy..."
# Install ROS 2 using unified cross-platform installer
$(dirname "$0")/install_ros2_jazzy.sh

echo "Installing ROS 2 desktop packages..."
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    python3-colcon-common-extensions \
    python3-rosdep

echo "Installing Gazebo Garden..."
# Add Gazebo Garden repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-garden

# Initialize rosdep
if [ ! -d /etc/ros/rosdep ]; then
    sudo rosdep init
fi
rosdep update

echo ""
echo "🔧 Step 2: Installing system dependencies..."
sudo apt install -y \
    portaudio19-dev \
    python3-dev \
    build-essential \
    libasound2-dev \
    libffi-dev \
    libssl-dev \
    pkg-config \
    git \
    curl \
    vim \
    htop \
    tree

echo ""
echo "🐍 Step 3: Setting up Python environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo "Created Python virtual environment"
else
    echo "Python virtual environment already exists"
fi

# Activate virtual environment
source venv/bin/activate

echo "Installing Python development dependencies..."
pip install --upgrade pip setuptools wheel
pip install -r requirements-dev.txt

echo ""
echo "🏗️  Step 4: Building ROS workspace..."
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace with limited parallel jobs for laptops
echo "Building ROS packages..."
colcon build --symlink-install --packages-ignore-regex ".*venv.*|.*test.*|.*mock.*"

# Source workspace
source install/setup.bash

echo ""
echo "⚙️  Step 5: Configuring environment..."

# Add ROS sourcing to bashrc if not already present
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "Added ROS 2 sourcing to ~/.bashrc"
fi

# Add workspace sourcing to bashrc
WORKSPACE_SOURCE="source $(pwd)/install/setup.bash"
if ! grep -q "$WORKSPACE_SOURCE" ~/.bashrc; then
    echo "$WORKSPACE_SOURCE" >> ~/.bashrc
    echo "Added workspace sourcing to ~/.bashrc"
fi

# Add ROS domain ID
if ! grep -q "export ROS_DOMAIN_ID=42" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "Added ROS_DOMAIN_ID to ~/.bashrc"
fi

echo ""
echo "🧪 Step 6: Running basic tests..."

# Test ROS 2 installation
echo "Testing ROS 2 installation..."
if source /opt/ros/jazzy/setup.bash && ros2 --help > /dev/null 2>&1; then
    echo "✅ ROS 2 command line tools working"
else
    echo "❌ ROS 2 command line tools not working"
    exit 1
fi

# Test workspace build
echo "Testing workspace build..."
if [ -f "install/setup.bash" ]; then
    echo "✅ Workspace built successfully"
else
    echo "❌ Workspace build failed"
    exit 1
fi

# Test Python environment
echo "Testing Python environment..."
if python3 -c "import cv2, numpy, torch" > /dev/null 2>&1; then
    echo "✅ Core Python packages installed"
else
    echo "❌ Some Python packages missing"
fi

echo ""
echo "🎉 Desktop Development Station Setup Complete!"
echo ""
echo "📋 Next Steps:"
echo "1. Open a new terminal (to load environment variables)"
echo "2. Test simulation (no venv needed for ROS/Gazebo):"
echo "   cd $(pwd)"
echo "   ros2 launch sim/launch/emu_gazebo_garden.launch.py"
echo ""
echo "3. Test vision processing (venv recommended for Python nodes):"
echo "   source venv/bin/activate"
echo "   ros2 launch emu_vision emu_vision_launch.py simulation:=true"
echo ""
echo "4. Monitor activity (no venv needed):"
echo "   ros2 topic echo /emu/report"
echo ""
echo "💡 Virtual Environment Usage:"
echo "   - ROS 2 commands: No venv needed (ros2 launch, ros2 topic, etc.)"
echo "   - Python development: Use venv (source venv/bin/activate)"
echo "   - Mixed workflows: Activate venv for consistency"
echo ""
echo "📚 Documentation:"
echo "   - README.md - Full installation guide"
echo "   - docs/quick_start_by_environment.md - Environment-specific guides"
echo "   - docs/network_setup.md - Multi-environment networking"
echo ""
echo "🔧 Development Tools:"
echo "   - code . (VSCode)"
echo "   - rqt_graph (ROS graph visualization)"
echo "   - rviz2 (3D visualization)"
echo ""

# Final environment check
echo "Current environment status:"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set}"
echo "  Python virtual environment: $(which python3)"
echo "  ROS 2 distro: ${ROS_DISTRO:-not sourced}"
echo ""
echo "✨ Happy developing! ✨"
