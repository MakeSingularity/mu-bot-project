#!/bin/bash
# Laptop Development Environment Setup Script
# Automated setup for portable development with power optimizations

set -e  # Exit on any error

echo "💻 Emu Droid Laptop Development Environment Setup"
echo "==============================================="
echo ""

# Prerequisites check
echo "🔍 Checking prerequisites..."

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "❌ Please do not run this script as root"
    echo "   Run as a regular user with sudo access"
    exit 1
fi

# Check sudo access
if ! sudo -n true 2>/dev/null; then
    echo "🔐 This script requires sudo access. Please enter your password when prompted."
    sudo -v
fi

# Check internet connectivity
if ! ping -c 1 google.com &> /dev/null; then
    echo "❌ No internet connection detected"
    echo "   Please check your network connection and try again"
    exit 1
fi

# Check available disk space (at least 5GB free)
available_space=$(df . | awk 'NR==2 {print $4}')
if [ "$available_space" -lt 5242880 ]; then # 5GB in KB
    echo "⚠️  Warning: Low disk space detected"
    echo "   Available: $(df -h . | awk 'NR==2 {print $4}')"
    echo "   Recommended: At least 5GB free space"
    read -p "Continue anyway? (y/N): " -r
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "✅ Prerequisites check passed"
echo ""

# Check if running on Ubuntu 24.04
if ! grep -q "noble" /etc/os-release; then
    echo "⚠️  Warning: This script is designed for Ubuntu 24.04 LTS (Noble)"
    echo "Your system may be different. Continue? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Exiting..."
        exit 1
    fi
fi

echo "📦 Step 1: Installing ROS 2 Jazzy..."
# Install ROS 2 using unified cross-platform installer
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/install_ros2_jazzy.sh"

echo "Installing ROS 2 desktop packages..."
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-controller-manager \
    python3-colcon-common-extensions \
    python3-rosdep

echo "Gazebo Sim 8 is included with ROS 2 Jazzy desktop packages..."
# Note: ROS 2 Jazzy includes ros-jazzy-gz-sim-vendor which provides Gazebo Sim 8 (Harmonic)
# No need to install separate Gazebo packages as they're included with ros-jazzy-ros-gz

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
    python3-venv \
    python3-pip \
    python3.12-venv \
    build-essential \
    libasound2-dev \
    libffi-dev \
    libssl-dev \
    pkg-config \
    git \
    curl \
    vim \
    htop \
    tree \
    powertop \
    tlp \
    tlp-rdw

echo ""
echo "⚡ Step 3: Configuring power management for laptops..."
# Enable TLP for better battery life during development
sudo systemctl enable tlp
sudo systemctl start tlp

# Configure power-saving settings
echo "Configuring laptop-specific power optimizations..."
sudo tee /etc/tlp.d/99-emu-droid-laptop.conf > /dev/null << EOF
# Emu Droid Laptop Power Optimization
# Optimized for development work with good battery life

# CPU frequency scaling
CPU_SCALING_GOVERNOR_ON_AC=performance
CPU_SCALING_GOVERNOR_ON_BAT=powersave
CPU_ENERGY_PERF_POLICY_ON_AC=performance
CPU_ENERGY_PERF_POLICY_ON_BAT=power

# Disable turbo boost on battery for better thermal management
CPU_BOOST_ON_AC=1
CPU_BOOST_ON_BAT=0

# Reduce CPU max frequency on battery
CPU_SCALING_MAX_FREQ_ON_AC=0
CPU_SCALING_MAX_FREQ_ON_BAT=2000000

# GPU power management
RADEON_POWER_PROFILE_ON_AC=high
RADEON_POWER_PROFILE_ON_BAT=low

# WiFi power saving
WIFI_PWR_ON_AC=off
WIFI_PWR_ON_BAT=on

# USB power management
USB_AUTOSUSPEND=1
USB_BLACKLIST_PHONE=1

# Enable laptop mode for disk I/O
DISK_APM_LEVEL_ON_AC="254 254"
DISK_APM_LEVEL_ON_BAT="128 128"
EOF

echo ""
echo "🐍 Step 4: Setting up Python environment..."

# Clean up any corrupted virtual environment
if [ -d "venv" ] && [ ! -f "venv/bin/activate" ]; then
    echo "Removing corrupted virtual environment..."
    rm -rf venv
fi

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv
    if [ $? -ne 0 ]; then
        echo "❌ Failed to create virtual environment. Checking Python installation..."
        python3 --version
        which python3
        echo "Trying alternative venv creation method..."
        /usr/bin/python3 -m venv venv
    fi
    echo "✅ Created Python virtual environment"
else
    echo "✅ Python virtual environment already exists"
fi

# Verify virtual environment was created successfully
if [ ! -f "venv/bin/activate" ]; then
    echo "❌ Virtual environment creation failed - activate script not found"
    echo "Python version: $(python3 --version)"
    echo "Available Python packages:"
    dpkg -l | grep python3-venv
    exit 1
fi

# Activate virtual environment
source venv/bin/activate

echo "Installing Python development dependencies..."
pip install --upgrade pip setuptools wheel
pip install -r requirements-dev.txt

echo ""
echo "🏗️  Step 5: Building ROS workspace..."
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace with limited parallel jobs for laptops
echo "Building ROS packages (laptop-optimized)..."
# Detect CPU cores and use 75% for build (leaves some for system)
CORES=$(nproc)
BUILD_JOBS=$((CORES * 3 / 4))
if [ $BUILD_JOBS -lt 1 ]; then
    BUILD_JOBS=1
fi
echo "Using $BUILD_JOBS parallel build jobs (detected $CORES cores)"

# Build only packages in src/ directory to avoid venv conflicts
colcon build --base-paths src --symlink-install --parallel-workers $BUILD_JOBS

# Create symlinks for ROS 2 launch compatibility
if [ -d "install/emu_vision/bin" ]; then
    mkdir -p install/emu_vision/lib/emu_vision
    ln -sf ../../bin/emu_observer install/emu_vision/lib/emu_vision/ 2>/dev/null || true
    ln -sf ../../bin/emu_tracker install/emu_vision/lib/emu_vision/ 2>/dev/null || true
    ln -sf ../../bin/emu_pose_estimator install/emu_vision/lib/emu_vision/ 2>/dev/null || true
fi

# Source workspace
source install/setup.bash

echo ""
echo "⚙️  Step 6: Configuring environment..."

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

# Add laptop-specific environment variables
if ! grep -q "export EMU_ENVIRONMENT=laptop" ~/.bashrc; then
    echo "export EMU_ENVIRONMENT=laptop" >> ~/.bashrc
    echo "Added environment identifier to ~/.bashrc"
fi

# Add RMW implementation setting
if ! grep -q "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" ~/.bashrc; then
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
    echo "Added RMW_IMPLEMENTATION to ~/.bashrc"
fi

# Configure Git if not already configured
if ! git config user.name > /dev/null 2>&1; then
    echo "Configuring Git user settings..."
    git config user.name "makesingularity"
    git config user.email "dev@makesingularity.org"
    echo "Git user configuration completed"
fi

echo ""
echo "🌐 Step 7: Configuring networking for remote development..."

# Create network configuration helper
cat > ~/emu_network_config.sh << 'EOF'
#!/bin/bash
# Emu Droid Laptop Network Configuration Helper
# Use this to connect to desktop or droid for remote development

echo "🌐 Emu Droid Network Configuration"
echo "Select your target environment:"
echo "1) Desktop development station"
echo "2) Raspberry Pi droid"
echo "3) Standalone laptop mode"
echo "4) Show current configuration"

read -p "Enter choice (1-4): " choice

case $choice in
    1)
        read -p "Enter desktop IP address: " desktop_ip
        echo "export ROS_DISCOVERY_SERVER=$desktop_ip:11811" >> ~/.bashrc
        echo "✅ Configured to connect to desktop at $desktop_ip"
        echo "Restart terminal or run: source ~/.bashrc"
        ;;
    2)
        read -p "Enter Raspberry Pi IP address: " pi_ip
        echo "export ROS_DISCOVERY_SERVER=$pi_ip:11811" >> ~/.bashrc
        echo "✅ Configured to connect to Pi at $pi_ip"
        echo "Restart terminal or run: source ~/.bashrc"
        ;;
    3)
        # Remove discovery server setting
        sed -i '/ROS_DISCOVERY_SERVER/d' ~/.bashrc
        echo "✅ Configured for standalone mode"
        echo "Restart terminal or run: source ~/.bashrc"
        ;;
    4)
        echo "Current configuration:"
        echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set}"
        echo "  ROS_DISCOVERY_SERVER: ${ROS_DISCOVERY_SERVER:-not set}"
        echo "  EMU_ENVIRONMENT: ${EMU_ENVIRONMENT:-not set}"
        ;;
    *)
        echo "Invalid choice"
        ;;
esac
EOF

chmod +x ~/emu_network_config.sh
echo "Created network configuration helper: ~/emu_network_config.sh"

echo ""
echo "🧪 Step 8: Running basic tests..."

# Test ROS 2 installation
echo "Testing ROS 2 installation..."
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if ros2 --help > /dev/null 2>&1; then
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

# Test power management
echo "Testing power management..."
if systemctl is-enabled tlp > /dev/null 2>&1; then
    echo "✅ Power management configured"
else
    echo "⚠️  Power management not fully configured"
fi

echo ""
echo "🎉 Laptop Development Environment Setup Complete!"
echo ""
echo "💻 Laptop-Specific Features:"
echo "✅ Power management optimized for battery life"
echo "✅ Parallel build jobs optimized for laptop CPUs"
echo "✅ Network configuration helper for remote development"
echo ""
echo "📋 Next Steps:"
echo "1. Open a new terminal (to load environment variables)"
echo "2. Configure network connection:"
echo "   ~/emu_network_config.sh"
echo ""
echo "3. Test display compatibility (important for WSL users):"
echo "   ./scripts/test_gazebo_display.sh"
echo ""
echo "4. Test local simulation:"
echo "   cd $(pwd)"
echo "   source venv/bin/activate"
echo "   ros2 launch sim/launch/emu_gazebo_sim8.launch.py"
echo "   # If GUI doesn't work, use: ros2 launch sim/launch/emu_gazebo_sim8.launch.py gui:=false"
echo ""
echo "5. Connect to remote droid:"
echo "   ros2 topic list  # Should show topics from remote system"
echo "   ros2 topic echo /emu/report  # Listen to droid reports"
echo ""
echo "📱 Power Management:"
echo "   - TLP configured for optimal battery usage"
echo "   - CPU scaling: Performance on AC, PowerSave on battery"
echo "   - Monitor with: sudo tlp-stat"
echo ""
echo "🌐 Remote Development:"
echo "   - Use ~/emu_network_config.sh to switch between environments"
echo "   - VSCode Remote-SSH for Pi development"
echo "   - ROS tools work across network transparently"
echo ""
echo "📚 Documentation:"
echo "   - README.md - Full installation guide"
echo "   - docs/quick_start_by_environment.md - Environment guides"
echo "   - docs/network_setup.md - Multi-environment networking"
echo ""

# Final environment check
echo "Current environment status:"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set}"
echo "  Python virtual environment: $(which python3)"
echo "  ROS 2 distro: ${ROS_DISTRO:-not sourced}"
echo "  Power management: $(systemctl is-enabled tlp 2>/dev/null || echo 'not configured')"
echo "  Build optimization: $BUILD_JOBS parallel jobs"
echo ""
echo "✨ Happy mobile developing! ✨"
