# Laptop Setup Guide - Gazebo Garden Edition

## Overview

This guide provides step-by-step instructions for setting up the Emu Droid development environment on your laptop, including the new Gazebo Garden simulation system.

## Prerequisites

- **OS**: Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- **Hardware**: 8GB+ RAM, 20GB+ free disk space
- **Network**: Internet connection for package downloads
- **Graphics**: Integrated graphics sufficient for development (discrete GPU optional)

## Automated Setup (Recommended)

### Quick Start
```bash
# 1. Clone the repository
git clone https://github.com/makesingularity/mu-bot-project.git
cd mu-bot-project

# 2. Run automated laptop setup
./scripts/setup_laptop.sh

# 3. Test Gazebo Garden compatibility
./scripts/test_gazebo_display.sh

# 4. Test simulation
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=false
```

## What the Setup Script Does

### 1. ROS 2 Installation
- Installs ROS 2 Humble desktop packages
- Configures ROS 2 environment and middleware
- Installs essential development tools

### 2. Gazebo Garden Installation
- Adds official Gazebo repositories
- Installs Gazebo Garden (latest stable version)
- Installs ROS 2 integration packages (`ros-humble-ros-gz`)

### 3. Python Environment
- Creates isolated virtual environment
- Installs development dependencies (no hardware packages)
- Configures colcon to exclude virtual environment

### 4. Power Management (Laptop-Specific)
- Installs and configures TLP for battery optimization
- Sets up CPU scaling profiles
- Optimizes for portable development

### 5. Network Configuration
- Creates network switching helper script
- Configures ROS domain and discovery
- Sets up multi-environment compatibility

## Manual Verification Steps

### Test ROS 2 Installation
```bash
# Check ROS 2 environment
source /opt/ros/humble/setup.bash
ros2 --version
ros2 topic list

# Verify workspace
cd ~/mu-bot-project  # or wherever you cloned
source install/setup.bash
ros2 pkg list | grep emu
```

### Test Gazebo Garden
```bash
# Test display compatibility
./scripts/test_gazebo_display.sh

# Expected output:
# ✅ X11 display working
# ✅ OpenGL direct rendering available
# ✅ Gazebo headless mode working
# ✅ Gazebo GUI working with compatibility settings
```

### Test Simulation Launch
```bash
# Headless mode (always works)
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=false

# GUI mode (if supported)
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=true

# Monitor in another terminal
ros2 topic echo /joint_states
ros2 topic echo /clock
```

## Environment-Specific Considerations

### WSL Users
If you're running Ubuntu in WSL on Windows:

1. **Update WSL**: Ensure you have WSLg enabled
   ```bash
   # In Windows PowerShell
   wsl --update
   wsl --shutdown
   wsl
   ```

2. **Fix Graphics Permissions**:
   ```bash
   sudo chmod 700 /run/user/1000
   sudo usermod -a -G video $USER
   # Logout and login again
   ```

3. **Use Headless Mode**: WSL graphics can be unreliable
   ```bash
   ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=false
   ```

### Native Linux
Full GUI support should work out of the box:
```bash
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=true
```

### Virtual Machines
Depending on GPU passthrough:
- **With GPU**: Full GUI support
- **Without GPU**: Use headless mode + RViz for visualization

## Development Workflow

### 1. Daily Development
```bash
# Start your development session
cd ~/mu-bot-project
source venv/bin/activate
source install/setup.bash

# Start simulation
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=false

# In another terminal - start your development nodes
ros2 launch emu_vision emu_vision_launch.py
```

### 2. Code and Test Cycle
```bash
# Make changes to code
# Build workspace
colcon build --symlink-install

# Test changes
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=false
```

### 3. Remote Development
```bash
# Connect to Pi droid (when available)
~/emu_network_config.sh

# List remote topics
ros2 topic list

# Monitor remote robot
ros2 topic echo /emu/report
```

## Power Management

The laptop setup includes optimizations for battery life:

### TLP Configuration
```bash
# Check TLP status
sudo tlp-stat

# Battery profiles are automatically configured:
# - AC Power: Performance mode
# - Battery: Power-save mode
```

### Manual Power Control
```bash
# Force power-save mode
sudo tlp bat

# Force performance mode
sudo tlp ac

# Auto mode (default)
sudo tlp start
```

## Troubleshooting

### Common Issues

#### "Gazebo GUI doesn't start"
```bash
# Test compatibility first
./scripts/test_gazebo_display.sh

# Use headless mode
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=false

# Use RViz for visualization
ros2 run rviz2 rviz2
```

#### "ROS commands not found"
```bash
# Source ROS environment
source /opt/ros/humble/setup.bash

# Source workspace
cd ~/mu-bot-project
source install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/mu-bot-project/install/setup.bash" >> ~/.bashrc
```

#### "Python package errors"
```bash
# Activate virtual environment
cd ~/mu-bot-project
source venv/bin/activate

# Reinstall dependencies
pip install -r requirements-dev.txt
```

#### "Colcon build fails"
```bash
# Ensure venv is excluded
touch venv/COLCON_IGNORE

# Build only source packages
colcon build --base-paths src --symlink-install
```

### Performance Issues

#### High CPU Usage
- Use headless mode: `gui:=false`
- Reduce simulation frequency
- Close unnecessary applications

#### Memory Issues
- Monitor with `htop`
- Increase swap if needed
- Use lighter desktop environment

## Network Setup for Multi-Environment

### Configure for Remote Development
```bash
# Use the network helper script
~/emu_network_config.sh

# Available options:
# 1. Local development (default)
# 2. Connect to desktop workstation
# 3. Connect to Pi droid
# 4. Custom configuration
```

### Manual Network Configuration
```bash
# Set ROS domain for your environment
export ROS_DOMAIN_ID=42  # Use your team's ID

# Set discovery peers (for known robots)
export ROS_DISCOVERY_SERVER=192.168.1.100:11811  # Pi droid IP
```

## Next Steps

1. **Learn the Codebase**: Explore `src/` directories
2. **Run Examples**: Test vision and control nodes
3. **Contribute**: Make improvements and submit PRs
4. **Deploy**: Transfer working code to Pi hardware

## Support

- **Issues**: Report bugs on GitHub
- **Discussions**: Join community discussions
- **Documentation**: Check `docs/` folder for more guides
- **Testing**: Use provided test scripts before reporting issues

---

*Setup guide updated for Gazebo Garden - October 17, 2025*
