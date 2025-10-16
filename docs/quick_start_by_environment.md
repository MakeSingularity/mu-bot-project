# Environment-Specific Quick Start Guide

This guide provides fast setup commands for each environment type in the Emu Droid project.

## 🖥️ Desktop Development Station

**Purpose:** Primary development, simulation, AI training
**Hardware:** Powerful desktop/workstation

### One-Command Setup
```bash
# Complete desktop setup (run in project directory)
./scripts/setup_desktop.sh
```

### Manual Setup Steps
```bash
# 1. ROS 2 installation
./scripts/fix_ros_keyring.sh
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions

# 2. System dependencies
sudo apt install portaudio19-dev python3-dev build-essential libasound2-dev libffi-dev libssl-dev pkg-config

# 3. Project setup
python3 -m venv venv
source venv/bin/activate
pip install -r requirements-dev.txt

# 4. Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Daily Development Workflow
```bash
# Start development session
cd mu-bot-project
source venv/bin/activate
source install/setup.bash

# Launch simulation
ros2 launch sim/launch/emu_gazebo.launch.py

# Start vision processing (new terminal)
ros2 launch emu_vision emu_vision_launch.py simulation:=true

# Monitor activity (new terminal)
rqt_graph  # Visualize node connections
ros2 topic echo /emu/report  # Listen to detection reports
```

---

## 💻 Laptop Development

**Purpose:** Portable development, field testing, remote debugging
**Hardware:** Standard laptop

### Setup (Same as Desktop)
Follow Desktop setup instructions above - requirements are identical.

### Field Testing Workflow
```bash
# Connect to robot network
export ROS_DOMAIN_ID=42
export ROS_DISCOVERY_SERVER=<droid_ip>:11811

# Monitor droid status
ros2 node list  # Check if droid nodes are running
ros2 topic list | grep emu  # See available emu topics

# Remote debugging
ros2 topic echo /emu/report  # Listen to droid reports
ros2 topic echo /emu/human_position  # Monitor tracking data
ros2 service call /emu/emergency_stop std_srvs/Empty  # Emergency control

# Performance monitoring
ros2 topic hz /emu/detection_image  # Check camera frame rate
ros2 topic bw /emu/detection_image  # Check bandwidth usage
```

### Remote Development with VSCode
```bash
# Install VSCode Remote-SSH extension
code --install-extension ms-vscode-remote.remote-ssh

# Connect to droid
code --remote ssh-remote+pi@<droid_ip> /home/pi/mu-bot-project

# Edit and test directly on hardware
```

---

## 🤖 Droid (Raspberry Pi 5)

**Purpose:** Deployed robot hardware with sensors and actuators
**Hardware:** Raspberry Pi 5 + HATs + sensors

### Hardware Setup Checklist
```bash
# 1. Verify hardware connections
i2cdetect -y 1  # Check I2C devices (should see PCA9685, Audio HAT)
lsusb  # Check USB devices (cameras)
gpio readall  # Check GPIO status

# 2. Test cameras
libcamera-hello --camera 0 --timeout 5000  # Left camera
libcamera-hello --camera 1 --timeout 5000  # Right camera

# 3. Test audio
arecord -f cd -t wav -d 3 test.wav  # Record test
aplay test.wav  # Playback test

# 4. Test servos (if connected)
python3 tests/test_servo_control.py
```

### Software Setup
```bash
# 1. ROS 2 installation (lightweight)
./scripts/fix_ros_keyring.sh
sudo apt install ros-humble-ros-base python3-colcon-common-extensions

# 2. Hardware dependencies
sudo apt install portaudio19-dev python3-dev build-essential i2c-tools gpio

# 3. Project setup
python3 -m venv venv
source venv/bin/activate
pip install -r requirements-pi.txt

# 4. Build workspace (memory-optimized)
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

### Operation Workflow
```bash
# Start robot operation
cd mu-bot-project
source venv/bin/activate
source install/setup.bash

# System health check
python3 tests/field_tests.py

# Start core systems
ros2 launch emu_vision emu_vision_launch.py hardware:=true

# Monitor performance (new terminal)
htop  # CPU/memory usage
ros2 topic hz /emu/report  # Detection frequency
journalctl -f -u robot  # System logs (if using systemd service)
```

### Maintenance Commands
```bash
# Update software
cd mu-bot-project
git pull
pip install -r requirements-pi.txt --upgrade
colcon build --symlink-install --parallel-workers 2

# System diagnostics
df -h  # Disk space
free -h  # Memory usage
vcgencmd measure_temp  # CPU temperature
vcgencmd get_throttled  # Throttling status

# Log rotation (prevent SD card wear)
sudo logrotate -f /etc/logrotate.conf
sudo journalctl --vacuum-size=100M
```

---

## 🌐 Multi-Environment Testing

### Network Communication Test
```bash
# On all systems - set domain
export ROS_DOMAIN_ID=42

# Desktop - run test publisher
ros2 run demo_nodes_cpp talker

# Laptop - verify node visibility
ros2 node list | grep talker

# Droid - listen to messages
ros2 topic echo /chatter
```

### End-to-End System Test
```bash
# Droid - start hardware detection
ros2 launch emu_vision emu_vision_launch.py hardware:=true

# Laptop - monitor detection reports
ros2 topic echo /emu/report

# Desktop - visualize in simulation
ros2 launch sim/launch/emu_gazebo.launch.py
rviz2 -d config/emu_multi_robot.rviz
```

### Performance Benchmarking
```bash
# Test detection latency
ros2 topic delay /emu/detection_image /emu/report

# Test network bandwidth
ros2 topic bw /emu/detection_image

# Test system load
# On Droid:
stress-ng --cpu 2 --timeout 60s &
ros2 topic hz /emu/report  # Should maintain >1Hz even under load
```

---

## 🚨 Emergency Procedures

### Robot Emergency Stop
```bash
# From any networked system
ros2 service call /emu/emergency_stop std_srvs/Empty

# Or directly on droid
pkill -f "ros2|python3.*emu"
```

### System Recovery
```bash
# Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# Restart robot software
cd mu-bot-project
source venv/bin/activate
source install/setup.bash
ros2 launch emu_vision emu_vision_launch.py hardware:=true
```

### Backup Important Data
```bash
# On droid - backup logs and config
rsync -av /home/pi/mu-bot-project/logs/ backup@desktop:/backups/droid-logs/
rsync -av /home/pi/mu-bot-project/config/ backup@desktop:/backups/droid-config/

# Backup learned models
rsync -av /home/pi/mu-bot-project/ai/models/ backup@desktop:/backups/droid-models/
```

---

## 📋 Environment Comparison

| Feature | Desktop | Laptop | Droid |
|---------|---------|---------|-------|
| **ROS 2 Install** | `ros-humble-desktop` | `ros-humble-desktop` | `ros-humble-ros-base` |
| **Python Packages** | `requirements-dev.txt` | `requirements-dev.txt` | `requirements-pi.txt` |
| **Build Workers** | Unlimited | Unlimited | 2 (memory limit) |
| **Primary Use** | Development/Simulation | Remote Development | Hardware Control |
| **Network Role** | Master/Simulation | Client/Monitor | Edge/Sensor |
| **Storage Needs** | High (models/logs) | Medium (development) | Low (constrained) |

## 🔧 Troubleshooting Quick Fixes

### "No module named 'RPi.GPIO'" on Desktop/Laptop
```bash
# Use development requirements instead
pip install -r requirements-dev.txt
```

### "Cannot connect to ROS master" across environments
```bash
# Check domain ID matches
echo $ROS_DOMAIN_ID  # Should be 42 on all systems

# Restart ROS daemon
ros2 daemon stop && ros2 daemon start
```

### "Permission denied" on GPIO/I2C (Droid)
```bash
sudo usermod -a -G gpio,i2c,spi $USER
# Logout and login again
```

### High CPU usage on Droid
```bash
# Reduce camera resolution
# Edit: src/emu_vision/config/emu_vision_config.yaml
# Set: camera_resolution: [640, 480]  # Instead of [1920, 1080]

# Reduce detection frequency
# Set: detection_rate: 2.0  # Instead of 10.0
```
