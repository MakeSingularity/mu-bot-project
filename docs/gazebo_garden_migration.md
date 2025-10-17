# Gazebo Garden Migration Guide

## Overview

This guide helps you migrate from Gazebo Classic (gazebo11) to Gazebo Garden for the Emu Droid project. Gazebo Classic reached end-of-life in January 2025, making this migration essential for continued support.

## Migration Status

✅ **Complete**: Migration to Gazebo Garden is now the default
🔄 **Transition**: Legacy Gazebo Classic files maintained for reference
📋 **Testing**: Use `./scripts/test_gazebo_display.sh` to verify compatibility

## Quick Migration Steps

### For Fresh Installations
```bash
# Use updated setup scripts (already include Gazebo Garden)
./scripts/setup_desktop.sh   # Desktop environment
./scripts/setup_laptop.sh    # Laptop environment
```

### For Existing Installations

#### 1. Remove Old Gazebo Classic
```bash
# Remove conflicting packages
sudo apt remove gazebo11 libgazebo11 gazebo-common gazebo-plugin-base
sudo apt autoremove

# Clean up old repositories (if needed)
sudo rm -f /etc/apt/sources.list.d/gazebo-classic.list
```

#### 2. Install Gazebo Garden
```bash
# Add Gazebo Garden repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install
sudo apt update
sudo apt install -y gz-garden

# Install ROS 2 integration
sudo apt install -y ros-humble-ros-gz
```

#### 3. Update Launch Commands
```bash
# Old (Gazebo Classic)
ros2 launch sim/launch/emu_gazebo.launch.py

# New (Gazebo Garden) - Recommended
ros2 launch sim/launch/emu_gazebo_garden.launch.py

# For headless mode (great for WSL)
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=false
```

## Key Differences

### Command Changes
| Gazebo Classic | Gazebo Garden |
|----------------|---------------|
| `gazebo` | `gz sim` |
| `ros-humble-gazebo-ros-pkgs` | `ros-humble-ros-gz` |
| `.world` files | `.sdf` files |
| `spawn_entity.py` | `ros_gz_sim/create` |

### File Updates
- **Launch files**: `emu_gazebo.launch.py` → `emu_gazebo_garden.launch.py`
- **World files**: `emu_testing_world.world` → `emu_testing_world.sdf`
- **Setup scripts**: Updated with Gazebo Garden installation

### WSL Compatibility
Gazebo Garden includes better WSL support:
- Improved graphics compatibility
- Software rendering fallbacks
- Headless mode for CI/CD pipelines

## Testing Your Migration

### 1. Test Display Compatibility
```bash
./scripts/test_gazebo_display.sh
```

### 2. Test Simulation
```bash
# Test headless mode first
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=false

# If headless works, try GUI
ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=true
```

### 3. Verify Robot Loading
```bash
# Check if robot model loads correctly
ros2 topic echo /robot_description

# Monitor joint states
ros2 topic echo /joint_states
```

## Troubleshooting

### GUI Issues in WSL
**Problem**: Gazebo Garden GUI crashes with segmentation fault
**Solutions**:
1. Use headless mode: `gui:=false`
2. Fix permissions: `sudo chmod 700 /run/user/1000`
3. Try software rendering environment variables
4. Use RViz for visualization instead

### Package Conflicts
**Problem**: "Conflicts with gazebo11" during installation
**Solution**: Remove old Gazebo completely first:
```bash
sudo apt remove --purge gazebo* libgazebo*
sudo apt autoremove
sudo apt autoclean
```

### Missing Dependencies
**Problem**: "ros_gz_sim not found"
**Solution**: Install ROS 2 Gazebo integration:
```bash
sudo apt install ros-humble-ros-gz
```

## Rollback Plan (If Needed)

If you encounter issues, you can temporarily use the legacy Gazebo Classic:
```bash
# Install legacy Gazebo Classic (NOT recommended for production)
sudo apt install gazebo11 ros-humble-gazebo-ros-pkgs

# Use legacy launch file
ros2 launch sim/launch/emu_gazebo.launch.py
```

**⚠️ Warning**: Gazebo Classic is end-of-life and should only be used for troubleshooting.

## Benefits of Migration

✅ **Long-term Support**: Gazebo Garden is actively maintained
✅ **Better Performance**: Improved physics and rendering
✅ **Modern Architecture**: Cleaner codebase and APIs
✅ **WSL Compatibility**: Better support for development environments
✅ **Future-Proofing**: Ensures compatibility with future ROS releases

## Support

If you encounter issues during migration:
1. Check the troubleshooting section above
2. Run the display test script: `./scripts/test_gazebo_display.sh`
3. Open an issue on GitHub with your error details
4. Include your OS version and environment type (desktop/laptop/WSL)

---

*Migration completed on October 17, 2025*
*Legacy support ends when Gazebo Classic reaches end-of-life (January 2025)*
