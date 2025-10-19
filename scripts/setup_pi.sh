#!/bin/bash
# Raspberry Pi Droid Hardware Setup Script
# Automated setup for Raspberry Pi 5 with Ubuntu 24.04 LTS and HAT stack
#
# PREREQUISITES:
# 1. Fresh Ubuntu 24.04 LTS Server installation on Pi 5
# 2. Internet connection
# 3. sudo access
#
# USAGE:
# sudo apt update && sudo apt install -y git
# git clone https://github.com/makesingularity/mu-bot-project.git
# cd mu-bot-project
# ./scripts/setup_pi.sh

set -e  # Exit on any error

echo "🤖 Emu Droid Raspberry Pi Hardware Setup"
echo "========================================"
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

# Check available disk space (at least 3GB free for Pi)
available_space=$(df . | awk 'NR==2 {print $4}')
if [ "$available_space" -lt 3145728 ]; then # 3GB in KB
    echo "⚠️  Warning: Low disk space detected"
    echo "   Available: $(df -h . | awk 'NR==2 {print $4}')"
    echo "   Recommended: At least 3GB free space"
    read -p "Continue anyway? (y/N): " -r
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "✅ Prerequisites check passed"
echo ""

# Install essential tools first (including git for repository cloning)
echo "📦 Installing essential system tools..."
echo "Note: Install git first so you can clone the repository with:"
echo "   git clone https://github.com/makesingularity/mu-bot-project.git"
echo "   cd mu-bot-project"
echo "   ./scripts/setup_pi.sh"
echo ""

sudo apt update
sudo apt install -y git curl wget

# Check if running on Raspberry Pi with Ubuntu 24.04
if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    echo "⚠️  Warning: This script is designed for Raspberry Pi"
    echo "Your system may be different. Continue? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Exiting..."
        exit 1
    fi
fi

# Check for Ubuntu 24.04 on Pi
if ! grep -q "noble" /etc/os-release 2>/dev/null; then
    echo "⚠️  Warning: This script is designed for Ubuntu 24.04 LTS (Noble) on Raspberry Pi"
    echo "Your system may be different. Continue? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Exiting..."
        exit 1
    fi
fi

echo "🔧 Step 1: Configuring Raspberry Pi interfaces..."

# Enable hardware interfaces for Ubuntu on Pi
echo "Configuring hardware interfaces for Ubuntu on Pi..."

# Create/update config.txt for hardware interfaces
CONFIG_FILE="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_FILE" ]; then
    # Fallback for older Ubuntu Pi images
    CONFIG_FILE="/boot/config.txt"
fi

# Enable I2C
if ! grep -q "dtparam=i2c_arm=on" "$CONFIG_FILE"; then
    echo "dtparam=i2c_arm=on" | sudo tee -a "$CONFIG_FILE"
    echo "Enabled I2C interface"
fi

# Enable SPI
if ! grep -q "dtparam=spi=on" "$CONFIG_FILE"; then
    echo "dtparam=spi=on" | sudo tee -a "$CONFIG_FILE"
    echo "Enabled SPI interface"
fi

# Enable Camera
if ! grep -q "start_x=1" "$CONFIG_FILE"; then
    echo "start_x=1" | sudo tee -a "$CONFIG_FILE"
    echo "Enabled Camera interface"
fi

# Configure SSH (should already be enabled in Ubuntu Server)
if systemctl is-active --quiet sshd; then
    echo "✅ SSH service already running"
elif systemctl is-active --quiet ssh; then
    echo "✅ SSH service already running"
else
    # Try sshd first (Ubuntu 24.04), then ssh (older versions)
    if systemctl enable sshd 2>/dev/null && systemctl start sshd 2>/dev/null; then
        echo "✅ SSH service (sshd) enabled and started"
    elif systemctl enable ssh 2>/dev/null && systemctl start ssh 2>/dev/null; then
        echo "✅ SSH service (ssh) enabled and started"
    else
        echo "⚠️  SSH service configuration failed - manual setup may be needed"
    fi
fi

echo "Hardware interfaces configured. Will require reboot later."

echo ""
echo "📦 Step 2: System update and ROS 2 installation..."
sudo apt update && sudo apt upgrade -y

# Install essential system packages first
sudo apt install -y \
    curl \
    gnupg \
    lsb-release \
    apt-transport-https \
    ca-certificates

# Note: software-properties-common not needed for Debian-based Pi OS

# Install ROS 2 using unified cross-platform installer
$(dirname "$0")/install_ros2_jazzy.sh

echo "Installing ROS 2 Jazzy (Pi-optimized)..."
# Install ROS base (lighter than desktop for Pi)
sudo apt install -y \
    ros-jazzy-ros-base \
    ros-jazzy-camera-info-manager \
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
if [ ! -d /etc/ros/rosdep ]; then
    sudo rosdep init
fi
rosdep update

echo ""
echo "🔧 Step 3: Installing hardware-specific packages..."
sudo apt install -y \
    portaudio19-dev \
    python3-dev \
    python3-venv \
    python3-pip \
    build-essential \
    libasound2-dev \
    libffi-dev \
    libssl-dev \
    pkg-config \
    i2c-tools \
    python3-smbus \
    python3-setuptools \
    python3-wheel \
    git \
    vim \
    htop \
    tree \
    screen \
    tmux

# Install camera utilities (Ubuntu packages - different names than Raspberry Pi OS)
echo "Installing camera utilities..."
if sudo apt install -y libcamera-tools libcamera0 2>/dev/null; then
    echo "✅ Camera utilities installed (libcamera-tools)"
elif sudo apt install -y libcamera-apps 2>/dev/null; then
    echo "✅ Camera utilities installed (libcamera-apps)"
else
    echo "⚠️  Camera utilities installation failed - trying alternative packages..."
    # Try individual packages that are more likely to exist
    sudo apt install -y \
        libcamera0 \
        libcamera-ipa \
        python3-libcamera 2>/dev/null || echo "⚠️  Some camera packages unavailable"
fi

# Install development headers if available
sudo apt install -y libcamera-dev 2>/dev/null || echo "⚠️  libcamera-dev not available"

# Install GPIO and hardware libraries (note: some packages may have different names on Ubuntu)
echo "Installing GPIO and hardware libraries..."
sudo apt install -y \
    python3-rpi.gpio \
    python3-gpiozero \
    pigpio \
    python3-pigpio || echo "⚠️  Some GPIO packages may not be available on Ubuntu"

echo ""
echo "💻 Step 4: Installing Visual Studio Code (Development Tools)..."

# Install desktop environment for VS Code GUI (if not already installed)
echo "Installing minimal desktop environment for VS Code GUI..."
sudo apt install -y ubuntu-desktop-minimal

# Install VS Code for ARM64 Ubuntu
echo "Installing VS Code for ARM64 architecture..."

# Check if VS Code is already installed
if command -v code > /dev/null 2>&1; then
    echo "✅ VS Code already installed"
else
    # Clean up any existing Microsoft repositories and keys to avoid conflicts
    echo "Cleaning up any existing Microsoft repositories..."
    sudo rm -f /etc/apt/sources.list.d/vscode.list
    sudo rm -f /etc/apt/trusted.gpg.d/packages.microsoft.gpg
    sudo rm -f /usr/share/keyrings/microsoft.gpg

    # Download and install Microsoft GPG key for VS Code
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
    sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
    sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'

    # Update package cache and install VS Code
    sudo apt update
    sudo apt install -y code

    echo "✅ VS Code installed successfully"
fi

# Install recommended VS Code extensions
echo "Installing recommended VS Code extensions for Pi development..."
EXTENSIONS=(
    "github.copilot"
    "github.copilot-chat"
    "ms-python.python"
    "ms-python.flake8"
    "ms-python.black-formatter"
    "ms-vscode.cpptools"
    "ms-vscode.cmake-tools"
    "twxs.cmake"
    "redhat.vscode-yaml"
    "ms-vscode.vscode-json"
    "streetsidesoftware.code-spell-checker"
    "ms-python.pylint"
    "ms-toolsai.jupyter"
)

# Check if code command is accessible before installing extensions
if command -v code > /dev/null 2>&1; then
    for ext in "${EXTENSIONS[@]}"; do
        if code --list-extensions 2>/dev/null | grep -q "^$ext$"; then
            echo "✅ Extension $ext already installed"
        else
            echo "Installing extension: $ext"
            if code --install-extension "$ext" --force 2>/dev/null; then
                echo "✅ Installed $ext"
            else
                echo "⚠️  Failed to install $ext (may need manual installation)"
            fi
        fi
    done
else
    echo "⚠️  VS Code command not accessible - extensions will need manual installation"
fi

echo "   GUI Mode: Connect monitor/VNC and run 'code .'"
echo "   SSH Mode: Use VS Code Remote-SSH from desktop/laptop"
echo "   Terminal Mode: Use 'code --help' for CLI options"

echo ""
echo "🎵 Step 5: Configuring audio system..."

# Configure audio for WM8960 HAT (Ubuntu may need different setup)
echo "Setting up WM8960 Audio HAT for Ubuntu..."

# Note: Ubuntu on Pi may require different audio setup than Raspberry Pi OS
# Check if the WM8960 device tree overlay is supported
CONFIG_FILE="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/boot/config.txt"
fi

if ! grep -q "dtoverlay=seeed-2mic-voicecard" "$CONFIG_FILE"; then
    echo "dtoverlay=seeed-2mic-voicecard" | sudo tee -a "$CONFIG_FILE"
    echo "Added WM8960 device tree overlay"
fi

# Configure ALSA for WM8960 (may need adjustment for Ubuntu)
sudo tee /etc/asound.conf > /dev/null << 'EOF'
pcm.!default {
    type asym
    playback.pcm "plughw:seeed2micvoicec"
    capture.pcm "plughw:seeed2micvoicec"
}
ctl.!default {
    type hw
    card seeed2micvoicec
}
EOF

echo ""
echo "📷 Step 6: Configuring camera system..."

# Configure camera settings for stereo vision
# Use the correct config.txt path for Ubuntu on Pi
CONFIG_FILE="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/boot/config.txt"
fi

# Add camera configuration if not already present
if ! grep -q "# Emu Droid Stereo Camera Configuration" "$CONFIG_FILE"; then
    sudo tee -a "$CONFIG_FILE" > /dev/null << 'EOF'

# Emu Droid Stereo Camera Configuration
# Optimized for dual cameras on Ubuntu Pi

# Enable camera and allocate GPU memory
start_x=1
gpu_mem=128

# Camera interface settings
camera_auto_detect=1

# Increase USB current limit for multiple devices
max_usb_current=1

# Optimize for computer vision workload
gpu_freq=500
arm_freq=2000
over_voltage=2
EOF
    echo "Added camera configuration to $CONFIG_FILE"
else
    echo "Camera configuration already present"
fi

echo ""
echo "🔌 Step 7: Configuring I2C and GPIO..."

# Set I2C baudrate for faster communication with servos
CONFIG_FILE="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/boot/config.txt"
fi

if ! grep -q "dtparam=i2c_arm_baudrate" "$CONFIG_FILE"; then
    echo "dtparam=i2c_arm_baudrate=400000" | sudo tee -a "$CONFIG_FILE"
    echo "Configured I2C for 400kHz operation"
fi

# Configure GPIO for servo control and sensors
echo "Setting up GPIO permissions..."
sudo usermod -a -G gpio,i2c,spi,dialout $USER

# Create GPIO configuration
sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null << 'EOF'
# GPIO access for Emu Droid project
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0664"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0664"
SUBSYSTEM=="spidev", GROUP="spi", MODE="0664"
EOF

echo ""
echo "🐍 Step 8: Setting up Python environment..."

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

echo "Installing Python hardware dependencies..."
pip install --upgrade pip setuptools wheel

# Install Pi-specific requirements
pip install -r requirements-pi.txt

echo ""
echo "🏗️  Step 9: Building ROS workspace (Pi-optimized)..."
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace with limited parallel jobs (Pi memory constraint)
echo "Building ROS packages (memory-optimized for Pi)..."
# Use only 1-2 cores to prevent memory exhaustion
CORES=$(nproc)
if [ $CORES -ge 4 ]; then
    BUILD_JOBS=2
else
    BUILD_JOBS=1
fi
echo "Using $BUILD_JOBS parallel build jobs (detected $CORES cores)"

# Set memory limits for compilation
export MAKEFLAGS="-j$BUILD_JOBS"

colcon build --symlink-install --parallel-workers $BUILD_JOBS --packages-ignore-regex ".*venv.*|.*test.*|.*mock.*"

# Source workspace
source install/setup.bash

echo ""
echo "⚙️  Step 10: Configuring environment..."

# Add ROS sourcing to bashrc
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

# Add Pi-specific environment variables
if ! grep -q "export EMU_ENVIRONMENT=droid" ~/.bashrc; then
    echo "export EMU_ENVIRONMENT=droid" >> ~/.bashrc
    echo "Added environment identifier to ~/.bashrc"
fi

# Configure RMW for Pi efficiency
if ! grep -q "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" ~/.bashrc; then
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
    echo "Added efficient RMW implementation for Pi"
fi

echo ""
echo "🌐 Step 11: Network and services configuration..."

# Create systemd service for emu droid auto-start
sudo tee /etc/systemd/system/emu-droid.service > /dev/null << EOF
[Unit]
Description=Emu Droid Hardware Service
After=network.target

[Service]
Type=forking
User=pi
WorkingDirectory=$(pwd)
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source $(pwd)/install/setup.bash && source $(pwd)/venv/bin/activate && ros2 launch emu_vision emu_vision_launch.py hardware:=true'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

echo "Created systemd service (not enabled by default)"
echo "To enable auto-start: sudo systemctl enable emu-droid.service"

# Configure static IP (optional)
cat > ~/configure_static_ip.sh << 'EOF'
#!/bin/bash
# Configure static IP for reliable robot networking

echo "🌐 Configure Static IP for Emu Droid"
echo "Current network configuration:"
ip addr show | grep inet

echo ""
read -p "Do you want to configure a static IP? (y/N): " response
if [[ "$response" =~ ^[Yy]$ ]]; then
    read -p "Enter desired IP address (e.g., 192.168.1.100): " static_ip
    read -p "Enter gateway IP (e.g., 192.168.1.1): " gateway_ip
    read -p "Enter network interface (e.g., wlan0): " interface

    # Backup existing configuration
    sudo cp /etc/dhcpcd.conf /etc/dhcpcd.conf.backup

    # Add static IP configuration
    sudo tee -a /etc/dhcpcd.conf > /dev/null << EOL

# Emu Droid Static IP Configuration
interface $interface
static ip_address=$static_ip/24
static routers=$gateway_ip
static domain_name_servers=$gateway_ip 8.8.8.8
EOL

    echo "✅ Static IP configured: $static_ip"
    echo "⚠️  Reboot required to apply network changes"
else
    echo "Keeping DHCP configuration"
fi
EOF

chmod +x ~/configure_static_ip.sh
echo "Created network configuration helper: ~/configure_static_ip.sh"

echo ""
echo "🧪 Step 12: Hardware testing..."

echo "Testing I2C devices..."
if command -v i2cdetect >/dev/null 2>&1; then
    echo "I2C bus scan:"
    i2cdetect -y 1 || echo "No I2C devices detected yet (HATs may not be connected)"
    echo "✅ I2C tools working"
else
    echo "❌ I2C tools not installed properly"
fi

echo "Testing GPIO access..."
if [ -w /dev/gpiomem ]; then
    echo "✅ GPIO access configured"
else
    echo "⚠️  GPIO access may require group membership (logout/login needed)"
fi

echo "Testing camera access..."
if command -v libcamera-hello >/dev/null 2>&1; then
    echo "✅ Camera tools installed (libcamera-hello available)"
    echo "Test cameras with: libcamera-hello --list-cameras"
elif command -v libcamera-vid >/dev/null 2>&1; then
    echo "✅ Camera tools installed (libcamera-vid available)"
    echo "Test cameras with: libcamera-vid --list-cameras"
elif dpkg -l | grep -q libcamera; then
    echo "✅ Basic camera libraries installed"
    echo "Note: Command-line tools may need separate installation"
else
    echo "⚠️  Camera tools not available - may need manual installation"
    echo "   Try: sudo apt install libcamera-tools"
fi

echo "Testing ROS 2 installation..."
if ros2 --help > /dev/null 2>&1; then
    echo "✅ ROS 2 command line tools working"
else
    echo "❌ ROS 2 command line tools not working"
    exit 1
fi

echo "Testing workspace build..."
if [ -f "install/setup.bash" ]; then
    echo "✅ Workspace built successfully"
else
    echo "❌ Workspace build failed"
    exit 1
fi

echo "Testing Python environment..."
if python3 -c "import cv2, numpy, RPi.GPIO" > /dev/null 2>&1; then
    echo "✅ Core Python packages with hardware support installed"
else
    echo "⚠️  Some Python packages may be missing (hardware not connected?)"
fi

echo ""
echo "🎉 Raspberry Pi Droid Setup Complete!"
echo ""
echo "🤖 Hardware-Specific Features:"
echo "✅ I2C, SPI, Camera interfaces enabled"
echo "✅ WM8960 Audio HAT configured"
echo "✅ GPIO permissions set up"
echo "✅ Memory-optimized ROS build"
echo "✅ Pi-efficient RMW implementation"
echo "✅ Auto-start service created (disabled)"
echo "✅ Visual Studio Code installed for development"
echo ""
echo "📋 Next Steps:"
echo "1. 🔄 REBOOT REQUIRED for hardware interface changes:"
echo "   sudo reboot"
echo ""
echo "2. After reboot, test hardware:"
echo "   cd $(pwd)"
echo "   source venv/bin/activate"
echo "   python3 tests/hardware_validation.py"
echo ""
echo "3. Test cameras:"
echo "   libcamera-hello --list-cameras"
echo "   libcamera-hello --camera 0 --timeout 5000"
echo ""
echo "4. Open project in VS Code for development:"
echo "   code ."
echo "   # Install recommended extensions: Python, ROS, C/C++"
echo ""
echo "5. Start droid operation:"
echo "   ros2 launch emu_vision emu_vision_launch.py hardware:=true"
echo ""
echo "6. Monitor system resources:"
echo "   htop"
echo "   vcgencmd measure_temp"
echo ""
echo "🔧 Hardware Configuration:"
echo "   - I2C: 400kHz for fast servo communication"
echo "   - GPU memory: 128MB for camera processing"
echo "   - Audio: WM8960 HAT configured as default"
echo "   - Static IP: Run ~/configure_static_ip.sh (optional)"
echo ""
echo "🚀 Auto-Start (optional):"
echo "   sudo systemctl enable emu-droid.service"
echo "   sudo systemctl start emu-droid.service"
echo ""
echo "🌐 Networking:"
echo "   - ROS_DOMAIN_ID=42 for multi-environment communication"
echo "   - Efficient RMW implementation for Pi performance"
echo "   - Configure static IP for reliable robot networking"
echo ""
echo "📚 Documentation:"
echo "   - README.md - Full installation guide"
echo "   - hardware/wiring_guide.md - HAT connections"
echo "   - tests/ - Hardware validation scripts"
echo ""
echo "💻 Development Tools:"
echo "   - code . (VS Code - full IDE for Pi development)"
echo "   - ssh user@pi-ip (Remote development from desktop/laptop)"
echo "   - htop (system monitoring)"
echo "   - tmux/screen (persistent terminal sessions)"
echo ""

# Final environment check
echo "Current environment status:"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set}"
echo "  Python virtual environment: $(which python3)"
echo "  ROS 2 distro: ${ROS_DISTRO:-not sourced}"
echo "  EMU_ENVIRONMENT: ${EMU_ENVIRONMENT:-not set}"
echo "  Build optimization: $BUILD_JOBS parallel jobs"
echo "  Hardware interfaces: Check /boot/firmware/config.txt or /boot/config.txt"
echo ""
echo "⚠️  IMPORTANT: Reboot required for hardware changes to take effect!"
echo "After reboot: sudo reboot"
echo ""
echo "🤖 Ready to bring your emu droid to life! 🤖"
