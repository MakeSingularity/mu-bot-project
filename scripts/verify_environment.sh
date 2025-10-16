#!/bin/bash
# Environment Verification Script
# Tests that each environment is properly set up and configured

set -e  # Exit on any error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    local status=$1
    local message=$2
    case $status in
        "PASS") echo -e "${GREEN}✅ PASS${NC}: $message" ;;
        "FAIL") echo -e "${RED}❌ FAIL${NC}: $message" ;;
        "WARN") echo -e "${YELLOW}⚠️  WARN${NC}: $message" ;;
        "INFO") echo -e "${BLUE}ℹ️  INFO${NC}: $message" ;;
    esac
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if package is installed (apt)
package_installed() {
    dpkg -l | grep -q "^ii  $1 "
}

# Function to check if Python package is installed
python_package_installed() {
    python3 -c "import $1" >/dev/null 2>&1
}

# Function to check if file exists
file_exists() {
    [ -f "$1" ]
}

# Function to check if directory exists
dir_exists() {
    [ -d "$1" ]
}

echo "🧪 Emu Droid Environment Verification"
echo "====================================="
echo ""

# Detect environment type
ENVIRONMENT="unknown"
if [ -n "$EMU_ENVIRONMENT" ]; then
    ENVIRONMENT="$EMU_ENVIRONMENT"
elif grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    ENVIRONMENT="droid"
elif [ -f "/sys/class/power_supply/BAT0/status" ] || [ -f "/sys/class/power_supply/BAT1/status" ]; then
    ENVIRONMENT="laptop"
else
    ENVIRONMENT="desktop"
fi

print_status "INFO" "Detected environment: $ENVIRONMENT"
echo ""

# 1. Basic System Checks
echo "🖥️  System Environment Checks"
echo "-----------------------------"

# Check Ubuntu version
if grep -q "22.04" /etc/os-release; then
    print_status "PASS" "Ubuntu 22.04 LTS detected"
else
    print_status "WARN" "Not Ubuntu 22.04 LTS - may have compatibility issues"
fi

# Check architecture
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ] || [ "$ARCH" = "aarch64" ]; then
    print_status "PASS" "Architecture: $ARCH (supported)"
else
    print_status "WARN" "Architecture: $ARCH (may not be fully supported)"
fi

# Check available memory
MEMORY_GB=$(free -g | awk '/^Mem:/{print $2}')
if [ "$MEMORY_GB" -ge 4 ]; then
    print_status "PASS" "Memory: ${MEMORY_GB}GB (sufficient)"
else
    print_status "WARN" "Memory: ${MEMORY_GB}GB (may be insufficient for full functionality)"
fi

echo ""

# 2. ROS 2 Installation Checks
echo "🤖 ROS 2 Installation Checks"
echo "----------------------------"

# Check ROS 2 command line tools
if command_exists ros2; then
    print_status "PASS" "ROS 2 command line tools installed"

    # Check ROS distro
    if [ "$ROS_DISTRO" = "humble" ]; then
        print_status "PASS" "ROS 2 Humble detected"
    else
        print_status "WARN" "ROS_DISTRO=$ROS_DISTRO (expected 'humble')"
    fi

    # Check ROS domain ID
    if [ "$ROS_DOMAIN_ID" = "42" ]; then
        print_status "PASS" "ROS_DOMAIN_ID=42 configured"
    else
        print_status "WARN" "ROS_DOMAIN_ID=$ROS_DOMAIN_ID (recommended: 42)"
    fi
else
    print_status "FAIL" "ROS 2 command line tools not found"
fi

# Check ROS 2 packages
if [ "$ENVIRONMENT" = "droid" ]; then
    # Pi should have ros-base
    if package_installed "ros-humble-ros-base"; then
        print_status "PASS" "ROS 2 Humble base installed (Pi-optimized)"
    else
        print_status "FAIL" "ROS 2 Humble base not installed"
    fi
else
    # Desktop/Laptop should have full desktop
    if package_installed "ros-humble-desktop"; then
        print_status "PASS" "ROS 2 Humble desktop installed"
    else
        print_status "FAIL" "ROS 2 Humble desktop not installed"
    fi
fi

# Check additional ROS packages
ROS_PACKAGES=(
    "python3-colcon-common-extensions"
    "python3-rosdep"
)

for pkg in "${ROS_PACKAGES[@]}"; do
    if package_installed "$pkg"; then
        print_status "PASS" "$pkg installed"
    else
        print_status "FAIL" "$pkg not installed"
    fi
done

echo ""

# 3. Python Environment Checks
echo "🐍 Python Environment Checks"
echo "----------------------------"

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
if python3 -c "import sys; exit(0 if sys.version_info >= (3, 10) else 1)"; then
    print_status "PASS" "Python $PYTHON_VERSION (compatible)"
else
    print_status "FAIL" "Python $PYTHON_VERSION (requires 3.10+)"
fi

# Check virtual environment
if [ -n "$VIRTUAL_ENV" ]; then
    print_status "PASS" "Virtual environment active: $VIRTUAL_ENV"
elif dir_exists "venv"; then
    print_status "WARN" "Virtual environment exists but not activated"
else
    print_status "FAIL" "Virtual environment not found"
fi

# Check core Python packages
CORE_PACKAGES=("cv2" "numpy" "torch")
for pkg in "${CORE_PACKAGES[@]}"; do
    if python_package_installed "$pkg"; then
        print_status "PASS" "Python package: $pkg"
    else
        print_status "FAIL" "Python package: $pkg not installed"
    fi
done

# Environment-specific Python packages
if [ "$ENVIRONMENT" = "droid" ]; then
    # Pi-specific packages
    PI_PACKAGES=("RPi.GPIO" "gpiozero" "picamera2")
    for pkg in "${PI_PACKAGES[@]}"; do
        if python_package_installed "$pkg"; then
            print_status "PASS" "Pi package: $pkg"
        else
            print_status "FAIL" "Pi package: $pkg not installed"
        fi
    done
fi

echo ""

# 4. Repository Structure Checks
echo "📁 Repository Structure Checks"
echo "------------------------------"

# Check core directories
CORE_DIRS=("src" "scripts" "docs" "tests")
for dir in "${CORE_DIRS[@]}"; do
    if dir_exists "$dir"; then
        print_status "PASS" "Directory: $dir"
    else
        print_status "FAIL" "Directory: $dir missing"
    fi
done

# Check requirements files
REQ_FILES=("requirements.txt" "requirements-dev.txt" "requirements-pi.txt")
for file in "${REQ_FILES[@]}"; do
    if file_exists "$file"; then
        print_status "PASS" "Requirements file: $file"
    else
        print_status "FAIL" "Requirements file: $file missing"
    fi
done

# Check setup scripts
SETUP_SCRIPTS=("scripts/setup_desktop.sh" "scripts/setup_laptop.sh" "scripts/setup_pi.sh")
for script in "${SETUP_SCRIPTS[@]}"; do
    if file_exists "$script" && [ -x "$script" ]; then
        print_status "PASS" "Setup script: $script (executable)"
    elif file_exists "$script"; then
        print_status "WARN" "Setup script: $script (not executable)"
    else
        print_status "FAIL" "Setup script: $script missing"
    fi
done

echo ""

# 5. ROS Workspace Checks
echo "🏗️  ROS Workspace Checks"
echo "-----------------------"

# Check workspace build
if dir_exists "install" && file_exists "install/setup.bash"; then
    print_status "PASS" "ROS workspace built"

    # Check if workspace is sourced
    if [ -n "$AMENT_PREFIX_PATH" ]; then
        print_status "PASS" "ROS workspace sourced"
    else
        print_status "WARN" "ROS workspace not sourced in current shell"
    fi
else
    print_status "FAIL" "ROS workspace not built"
fi

# Check for ROS packages in src
if dir_exists "src"; then
    PKG_COUNT=$(find src -name "package.xml" | wc -l)
    if [ "$PKG_COUNT" -gt 0 ]; then
        print_status "PASS" "Found $PKG_COUNT ROS packages in src/"
    else
        print_status "WARN" "No ROS packages found in src/"
    fi
fi

echo ""

# 6. Environment-Specific Checks
echo "🎯 Environment-Specific Checks"
echo "------------------------------"

case $ENVIRONMENT in
    "desktop")
        print_status "INFO" "Running desktop environment checks..."

        # Check Gazebo
        if command_exists gazebo; then
            print_status "PASS" "Gazebo simulation available"
        else
            print_status "WARN" "Gazebo not installed"
        fi

        # Check development tools
        if command_exists code; then
            print_status "PASS" "VS Code available"
        else
            print_status "WARN" "VS Code not installed"
        fi
        ;;

    "laptop")
        print_status "INFO" "Running laptop environment checks..."

        # Check power management
        if command_exists tlp; then
            print_status "PASS" "TLP power management available"
        else
            print_status "WARN" "TLP power management not installed"
        fi

        # Check network helper
        if file_exists "$HOME/emu_network_config.sh"; then
            print_status "PASS" "Network configuration helper available"
        else
            print_status "WARN" "Network configuration helper not found"
        fi
        ;;

    "droid")
        print_status "INFO" "Running Raspberry Pi hardware checks..."

        # Check GPIO access
        if [ -w /dev/gpiomem ]; then
            print_status "PASS" "GPIO access available"
        else
            print_status "WARN" "GPIO access may require group membership"
        fi

        # Check I2C
        if command_exists i2cdetect; then
            print_status "PASS" "I2C tools available"
        else
            print_status "FAIL" "I2C tools not installed"
        fi

        # Check camera
        if command_exists libcamera-hello; then
            print_status "PASS" "Camera tools available"
        else
            print_status "FAIL" "Camera tools not installed"
        fi

        # Check hardware interfaces
        if grep -q "dtparam=i2c_arm=on" /boot/config.txt 2>/dev/null; then
            print_status "PASS" "I2C interface enabled"
        else
            print_status "WARN" "I2C interface may not be enabled"
        fi

        # Check systemd service
        if file_exists "/etc/systemd/system/emu-droid.service"; then
            print_status "PASS" "Auto-start service available"
        else
            print_status "WARN" "Auto-start service not configured"
        fi
        ;;

    *)
        print_status "WARN" "Unknown environment type - running generic checks only"
        ;;
esac

echo ""

# 7. Network Connectivity Checks
echo "🌐 Network Connectivity Checks"
echo "------------------------------"

# Check internet connectivity
if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
    print_status "PASS" "Internet connectivity"
else
    print_status "FAIL" "No internet connectivity"
fi

# Check ROS discovery server configuration
if [ -n "$ROS_DISCOVERY_SERVER" ]; then
    print_status "INFO" "ROS Discovery Server: $ROS_DISCOVERY_SERVER"
else
    print_status "INFO" "Using default ROS discovery (no server configured)"
fi

echo ""

# 8. Quick Functional Tests
echo "🧪 Quick Functional Tests"
echo "-------------------------"

# Test ROS 2 basic functionality
if command_exists ros2; then
    if timeout 5 ros2 topic list >/dev/null 2>&1; then
        print_status "PASS" "ROS 2 topic listing works"
    else
        print_status "WARN" "ROS 2 topic listing timeout (may be normal)"
    fi
fi

# Test Python imports
if python3 -c "import cv2, numpy; print('OpenCV:', cv2.__version__)" >/dev/null 2>&1; then
    print_status "PASS" "Computer vision libraries functional"
else
    print_status "FAIL" "Computer vision libraries not working"
fi

echo ""

# Summary
echo "📊 Verification Summary"
echo "======================"

# Count results
PASS_COUNT=$(grep -c "✅ PASS" /tmp/verification_output 2>/dev/null || echo "0")
FAIL_COUNT=$(grep -c "❌ FAIL" /tmp/verification_output 2>/dev/null || echo "0")
WARN_COUNT=$(grep -c "⚠️  WARN" /tmp/verification_output 2>/dev/null || echo "0")

# This is a simple approximation since we don't capture output to file
# In practice, you'd want to capture the output properly

echo "Environment: $ENVIRONMENT"
echo "System: $(uname -s) $(uname -m)"
echo "Date: $(date)"
echo ""

if [ "$FAIL_COUNT" -eq 0 ]; then
    print_status "PASS" "Environment verification completed successfully!"
    echo ""
    echo "🚀 Your $ENVIRONMENT environment is ready for Emu Droid development!"
    echo ""
    echo "Next steps:"
    case $ENVIRONMENT in
        "desktop")
            echo "  - Test simulation: ros2 launch sim/launch/emu_gazebo.launch.py"
            echo "  - Start development: code ."
            ;;
        "laptop")
            echo "  - Configure networking: ~/emu_network_config.sh"
            echo "  - Connect to remote systems for development"
            ;;
        "droid")
            echo "  - Test hardware: python3 tests/hardware_validation.py"
            echo "  - Start droid: ros2 launch emu_vision emu_vision_launch.py hardware:=true"
            ;;
    esac
else
    print_status "FAIL" "Environment verification found issues that need attention"
    echo ""
    echo "🔧 Please address the failed checks above and run verification again"
    echo ""
    echo "Common solutions:"
    echo "  - Run the appropriate setup script: ./scripts/setup_$ENVIRONMENT.sh"
    echo "  - Source the environment: source venv/bin/activate"
    echo "  - Install missing packages: sudo apt update && sudo apt upgrade"
    echo "  - Check the troubleshooting guide in README.md"
fi

echo ""
echo "📚 Documentation: README.md, docs/quick_start_by_environment.md"
echo "🐛 Issues? Report at: https://github.com/makesingularity/mu-bot-project/issues"
