# Emu Droid Companion Robot 🦆🤖

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![Hailo AI](https://img.shields.io/badge/Hailo_AI-26_TOPS-purple.svg)](https://hailo.ai/)

A bipedal companion robot inspired by an emu, designed for observe-and-report tasks using cutting-edge AI acceleration and open-source hardware.

## 🎯 Project Overview

The Emu Droid is an open-source bipedal companion robot designed to demonstrate affordable, accessible robotics using off-the-shelf components and 3D printing. Built around a Raspberry Pi 5 with Hailo AI acceleration, it can track humans walking or running at 1-5 m/s using stereo vision and report observations via natural speech.

**Target Demo**: 5 functional units walking at Mare Island Maker Faire - September 2026

### Key Features
- 🧠 **AI-Powered Vision**: Hailo AI HAT+ (26 TOPS) for real-time human detection and pose estimation
- 👀 **Stereo Vision**: Dual Arducam 5MP cameras with motorized autofocus
- 🗣️ **Voice Reporting**: WM8960 audio HAT for microphone input and TTS output
- 🦿 **Bipedal Design**: 2020 aluminum extrusion bones with 3D-printed joints
- ⚡ **Energy Efficient**: Configurable standby modes for extended operation
- 🔧 **Modular**: ROS 2 architecture with swappable components

## 🚀 Quick Start

## 🖥️ Multi-Environment Setup

The Emu Droid project is designed as a **distributed robotics system** with three types of environments that work together:

### 🎯 Environment Types

| Environment | Purpose | Hardware | Use Case |
|------------|---------|----------|----------|
| **🖥️ Desktop** | Primary development station | Powerful desktop/workstation | ROS development, simulation, AI training |
| **💻 Laptop** | Portable development | Standard laptop | Field programming, remote debugging |
| **🤖 Droid (RPi5)** | Robot hardware | Raspberry Pi 5 + HATs | Deployed robot with sensors/actuators |

### 🌐 How They Work Together

```
Desktop (ROS Master) ←→ Laptop (Remote Dev) ←→ Droid (Hardware)
     ↑                      ↑                     ↑
  Simulation           Field Testing         Real Robot
  AI Training         Remote Control        Human Tracking
  Development         Debugging             Audio Reports
```

---

## 🚀 Installation by Environment

Choose your installation path based on your current system:

### 📋 Prerequisites (All Environments)
- Ubuntu 22.04 LTS (native or WSL2)
- ROS 2 Humble Hawksbill
- Python 3.10 or higher
- Git and basic development tools

---

## 🖥️ Desktop Development Station Setup

**Use this for:** Primary development, simulation, AI model training

### 1. ROS 2 Installation (Required First!)

**⚠️ IMPORTANT: Install ROS 2 Humble AND system dependencies BEFORE running pip install!**

#### 🔧 Quick ROS 2 Installation
```bash
# Run our automated GPG keyring fix script
./scripts/fix_ros_keyring.sh

# Install ROS 2 Humble desktop (full stack)
sudo apt install ros-humble-desktop

# Install additional ROS 2 packages for development
sudo apt install \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 (add to ~/.bashrc for permanent setup)
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### 🐍 Python Environment Setup
```bash
# Clone the repository
git clone https://github.com/makesingularity/mu-bot-project.git
cd mu-bot-project

# Set up Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install system dependencies for Python packages
sudo apt install \
    portaudio19-dev \
    python3-dev \
    build-essential \
    libasound2-dev \
    libffi-dev \
    libssl-dev \
    pkg-config

# Install development dependencies (no hardware packages)
pip install -r requirements-dev.txt
```

#### 🏗️ Build ROS Workspace
```bash
# Source ROS 2 and build workspace
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

---

## 💻 Laptop Development Setup

**Use this for:** Portable development, field testing, remote debugging

### Installation (Same as Desktop)
Follow the **Desktop Development Station Setup** above - the requirements are identical for development environments.

### 🌐 Network Configuration for Remote Development
```bash
# Configure ROS 2 networking to connect to desktop or droid
export ROS_DOMAIN_ID=42  # Use same ID across all systems
export ROS_DISCOVERY_SERVER=<desktop_ip>:11811  # Optional: centralized discovery

# Add to ~/.bashrc for persistence
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

---

## 🤖 Droid (Raspberry Pi 5) Setup

**Use this for:** Deployed robot hardware with sensors and actuators

### 🔧 Hardware Prerequisites
- Raspberry Pi 5 (16GB recommended)
- Hailo AI HAT+ (26 TOPS)
- WM8960 Audio HAT
- PCA9685 PWM HAT (servo control)
- Arducam 5MP stereo cameras
- MicroSD card (64GB+, Class 10)

### 💾 Raspberry Pi OS Setup
```bash
# Flash Raspberry Pi OS (64-bit) to SD card
# Enable SSH and configure WiFi during flash

# SSH into Pi (replace with your Pi's IP)
ssh pi@<raspberry_pi_ip>

# Update system
sudo apt update && sudo apt upgrade -y

# Enable I2C, SPI, Camera interfaces
sudo raspi-config
# Interface Options -> Enable I2C, SPI, Camera
```

### 🔧 ROS 2 Installation on Pi
```bash
# Run our automated GPG keyring fix script
./scripts/fix_ros_keyring.sh

# Install ROS 2 Humble (lightweight for Pi)
sudo apt install ros-humble-ros-base  # Smaller footprint than desktop

# Install hardware-specific ROS packages
sudo apt install \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-vision-msgs \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 🐍 Python Environment (Pi Hardware)
```bash
# Clone repository
git clone https://github.com/makesingularity/mu-bot-project.git
cd mu-bot-project

# Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install hardware dependencies for Pi
sudo apt install \
    portaudio19-dev \
    python3-dev \
    build-essential \
    libasound2-dev \
    libffi-dev \
    libssl-dev \
    pkg-config \
    i2c-tools \
    gpio

# Install Pi-specific Python packages (includes hardware libraries)
pip install -r requirements-pi.txt
```

### 🌐 Network Configuration for Droid
```bash
# Configure ROS 2 networking
export ROS_DOMAIN_ID=42  # Match other systems
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp  # Efficient for Pi

# Configure as edge node (optional)
export ROS_DISCOVERY_SERVER=<desktop_ip>:11811

# Add to ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> ~/.bashrc
```

### ⚡ Build Workspace (Pi Optimized)
```bash
# Build with limited parallel jobs (Pi memory constraint)
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

---

## 🌐 Multi-Environment Network Setup

### 🎯 ROS 2 Domain Configuration

All systems should use the same domain ID for communication:

```bash
# Add to ~/.bashrc on ALL systems (Desktop, Laptop, Droid)
export ROS_DOMAIN_ID=42
```

### 🖥️ Desktop as ROS Master (Optional)
```bash
# On Desktop - set up discovery server
ros2 daemon stop
ros2 daemon start --ros-args -p discovery_server:=<desktop_ip>:11811

# On other systems, point to desktop
export ROS_DISCOVERY_SERVER=<desktop_ip>:11811
```

### 🔗 Testing Multi-Environment Communication
```bash
# On Desktop - publish test message
ros2 topic pub /test_topic std_msgs/String "data: Hello from Desktop"

# On Laptop - listen for message
ros2 topic echo /test_topic

# On Droid - check nodes are visible
ros2 node list
```

---

## 🚀 Quick Start by Environment

### 🖥️ Desktop: Start Simulation
```bash
source install/setup.bash

# Launch Gazebo simulation
ros2 launch sim/launch/emu_gazebo.launch.py

# Start vision processing (simulated)
ros2 launch emu_vision emu_vision_launch.py simulation:=true
```

### 💻 Laptop: Remote Development
```bash
source install/setup.bash

# Connect to droid and monitor
ros2 topic list  # See available topics from droid
ros2 topic echo /emu/report  # Listen to droid reports

# Launch development tools
code .  # Remote development with VSCode
```

### 🤖 Droid: Hardware Operation
```bash
source install/setup.bash

# Test hardware
python3 tests/field_tests.py

# Start droid operation
ros2 launch emu_vision emu_vision_launch.py hardware:=true

# Monitor system
htop  # Watch CPU/memory usage
```

---

## 📁 Repository Structure
```
```bash
# Install ROS 2 Humble (Ubuntu 22.04) - GPG Key Fix Included
sudo apt update && sudo apt install curl gnupg lsb-release

# IMPORTANT: Clean up any existing problematic ROS setup first
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Download ROS 2 GPG key to the correct location
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Verify the key was downloaded successfully
ls -la /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository with proper GPG key reference
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists (should work without GPG errors now)
sudo apt update

# If you still get GPG errors, use this alternative method:
# sudo apt install software-properties-common
# curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# sudo apt-add-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
# sudo apt update

# Install ROS 2 Humble desktop
sudo apt install ros-humble-desktop
```

---

#### Continue with Additional ROS 2 Packages (All users)
```bash
# Install additional ROS 2 packages
sudo apt install \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    python3-rosdep

# Install system dependencies for Python packages (prevents PyAudio compilation errors)
sudo apt install \
    portaudio19-dev \
    python3-dev \
    build-essential \
    libasound2-dev \
    libffi-dev \
    libssl-dev \
    pkg-config

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 (add to ~/.bashrc for permanent setup)
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. Repository Setup
```bash
# Clone the repository
git clone https://github.com/makesingularity/mu-bot-project.git
cd mu-bot-project

# Set up Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install system dependencies for Python packages (IMPORTANT!)
# These prevent PyAudio, librosa, and other audio package compilation errors
sudo apt install \
    portaudio19-dev \
    python3-dev \
    build-essential \
    libasound2-dev \
    libffi-dev \
    libssl-dev \
    pkg-config

# Install Python dependencies - Choose your environment:

# For DEVELOPMENT (laptops/desktops without hardware):
pip install -r requirements-dev.txt

# For DEPLOYMENT (Raspberry Pi with hardware):
pip install -r requirements-pi.txt

# Legacy (may fail on laptops due to hardware packages):
# pip install -r requirements.txt
```

### 3. ROS 2 Workspace Setup
```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Install workspace-specific dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build ROS packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add workspace to auto-source (optional)
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

### 4. Hardware Assembly (Optional)
For physical hardware deployment:

```bash
# Test Hailo AI HAT+ (requires physical hardware)
hailortcli scan

# Test cameras (requires Arducam modules)
libcamera-hello --camera 0 --timeout 5000
libcamera-hello --camera 1 --timeout 5000

# Test servo control (requires PCA9685 HAT)
python3 tests/test_servo_control.py

# Test audio (requires WM8960 HAT)
arecord -f cd -t wav -d 5 test.wav
aplay test.wav
```

### 5. Simulation (Gazebo)
```bash
# Launch simulation environment
ros2 launch sim/launch/emu_gazebo.launch.py

# Start vision processing
ros2 launch emu_vision emu_vision_launch.py

# Monitor human detection reports
ros2 topic echo /emu/report
```

## 📁 Repository Structure

```
mu-bot/
├── src/                        # ROS 2 packages
│   ├── emu_control/            # Motor control and gait planning
│   ├── emu_vision/             # Computer vision and AI inference
│   └── emu_audio/              # Audio processing and TTS
├── sim/                        # Gazebo simulation
│   ├── worlds/                 # Testing environments
│   ├── urdf/                   # Robot model definitions
│   └── launch/                 # Simulation launch files
├── scripts/                    # Utility and setup scripts
│   ├── fix_ros_keyring.sh      # GPG keyring fix script
│   └── setup_desktop.sh        # Automated desktop setup
├── ai/                         # AI model pipeline
│   ├── hailo/                  # Hailo model compilation
│   ├── pytorch/                # Custom model training
│   └── models/                 # Compiled model binaries
├── hardware/                   # Hardware documentation
│   ├── wiring_guide.md         # Complete wiring instructions
│   └── schematics.md           # Circuit diagrams and pinouts
├── docs/                       # Project documentation
│   ├── BOM.md                  # Bill of materials with pricing
│   ├── timeline.md             # Development timeline to 2026
│   ├── network_setup.md        # Multi-environment networking guide
│   └── quick_start_by_environment.md  # Environment-specific setup
├── tests/                      # Hardware and software tests
├── .vscode/                    # Development environment config
├── requirements.txt            # Python dependencies (environment selector)
├── requirements-dev.txt        # Python deps for development (Desktop/Laptop)
└── requirements-pi.txt         # Python deps for hardware (Raspberry Pi)
```
```

## 🔧 Hardware Components

### Core Computing Stack
- **Raspberry Pi 5 (16GB)** - Main computer brain
- **Hailo AI HAT+ (26 TOPS)** - AI acceleration for inference
- **Waveshare WM8960 Audio HAT** - Microphone input and speaker output
- **PCA9685 16-Channel PWM HAT** - Servo motor control

### Vision System
- **2x Arducam 5MP OV5647** - Stereo camera pair with autofocus
- **Stewart Platform Eye Mounts** - 3-DOF eye movement per camera
- **12cm Baseline** - Optimized for 1-10m depth perception

### Mechanical Structure
- **2020 Aluminum Extrusion** - Lightweight, strong bone structure
- **3D Printed Joints** - Custom connectors and housings
- **12x SG90 Servos** - Eye platforms (6) + neck control (6)
- **Cable-Driven Neck** - Flexible tentacle-like movement

**Total Build Cost: ~$935 per unit** (see [BOM.md](docs/BOM.md) for details)

## 🤖 Key Capabilities

### Human Observation & Tracking
- **Real-time Detection**: YOLOv8 models compiled for Hailo AI
- **Pose Estimation**: 17-keypoint human pose tracking at 20 FPS
- **Speed Calculation**: Velocity estimation from stereo depth
- **Distance Measurement**: 1-10 meter range with stereo cameras

### Activity Classification
- **Standing**: Stationary human detection and monitoring
- **Walking**: 1-3 m/s movement tracking and reporting
- **Running**: 3-5 m/s high-speed movement detection

### Voice Reporting
Example outputs:
- *"Human detected at 3.2 meters - walking at 2.1 meters per second"*
- *"Runner approaching - speed 4.7 meters per second, distance 8 meters"*
- *"No human activity detected - entering standby mode"*

### Energy Management
- **Active Mode**: 30 FPS vision processing (~25W total power)
- **Eco Mode**: 10 FPS processing (~15W total power)
- **Standby Mode**: 2 FPS monitoring (~8W total power)

## 🧠 AI Pipeline

### Model Compilation
```bash
# Compile all Hailo models
cd ai/hailo
python3 compile_hailo_models.py --all

# Verify model performance
hailortcli benchmark --model ../models/yolov8n.hef
```

### Custom Model Training
```bash
# Train activity classifier
cd ai/pytorch
python3 train_custom_models.py --model activity_classifier --dataset /path/to/data

# Export to ONNX for Hailo compilation
python3 train_custom_models.py --model activity_classifier --export-onnx
```

### Supported Models
- **YOLOv8n**: General object detection (11MB, 30 FPS)
- **YOLOv8n-pose**: Human pose estimation (6MB, 20 FPS)
- **Activity Classifier**: Custom walking/running detection (2MB, 50 FPS)
- **Distance Estimator**: Stereo depth estimation (4MB, 30 FPS)

## 🎮 Usage Examples

### Basic Operation
```bash
# Start complete emu droid system
ros2 launch emu_vision emu_vision_launch.py

# Monitor human detection
ros2 topic echo /emu/report

# Control head movement
ros2 topic pub /emu/head_cmd geometry_msgs/Twist '{angular: {z: 0.5}}'

# Emergency stop
ros2 service call /emu/emergency_stop std_srvs/Empty
```

### Development Testing
```bash
# Test individual components
python3 tests/test_camera_stereo.py
python3 tests/test_hailo_inference.py
python3 tests/test_audio_tts.py

# Calibrate stereo cameras
python3 tests/calibrate_stereo_cameras.py

# Run full system integration test
python3 tests/integration_test.py
```

### Simulation Environment
```bash
# Launch Gazebo world with human models
ros2 launch sim/launch/emu_gazebo.launch.py world:=emu_testing_world.world

# Control virtual human for testing
ros2 topic pub /human/cmd_vel geometry_msgs/Twist '{linear: {x: 2.0}}'

# Observe emu detection behavior
ros2 topic echo /emu/human_position
```

## 📊 Performance Specifications

| Metric | Target | Achieved |
|--------|--------|----------|
| Human Detection Accuracy | 95% | 🟡 In Progress |
| Pose Estimation FPS | 20 | 🟡 In Progress |
| Speed Tracking Error | ±0.2 m/s | 🟡 In Progress |
| Detection Range | 1-10 meters | 🟡 In Progress |
| Audio Response Time | <2 seconds | 🟡 In Progress |
| Battery Life (Future) | 4+ hours | 🔴 Not Started |

## 🛠️ Development Setup

### VSCode Configuration
The repository includes pre-configured VSCode settings for optimal development:

```bash
# Open in VSCode with ROS extensions
code .

# Extensions automatically suggested:
# - ROS 2 Extension Pack
# - Python Development Pack
# - C++ Development Pack
# - CMake Tools
```

### Remote Development (Raspberry Pi)
```bash
# Set up SSH key authentication
ssh-copy-id pi@your-emu-droid.local

# Configure VSCode Remote-SSH
# Host: emu-droid
# HostName: your-emu-droid.local
# User: pi
```

### Building and Testing
```bash
# Build all ROS packages
colcon build --symlink-install

# Run tests
colcon test
colcon test-result --all

# Format code
black src/
clang-format -i src/**/*.cpp
```

## 🧪 Testing Framework

### Hardware Tests
```bash
# Complete hardware validation
python3 tests/hardware_validation.py

# Individual component tests
python3 tests/test_pi_hat_stack.py      # GPIO and I2C
python3 tests/test_stereo_cameras.py    # Vision system
python3 tests/test_servo_control.py     # Motion control
python3 tests/test_audio_pipeline.py    # Sound input/output
```

### Software Tests
```bash
# ROS node tests
ros2 run emu_vision test_observer
ros2 run emu_control test_gait_planner
ros2 run emu_audio test_tts_engine

# Integration tests
python3 tests/test_vision_to_audio.py
python3 tests/test_multi_unit_coordination.py
```

### AI Model Tests
```bash
# Hailo inference benchmarks
python3 tests/benchmark_hailo_models.py

# Model accuracy validation
python3 tests/validate_human_detection.py
python3 tests/validate_pose_estimation.py
```

## 🔧 Troubleshooting

### Common Installation Issues

#### "Could not find a version that satisfies the requirement rclpy"
**Problem**: Trying to install ROS 2 packages via pip
**Solution**: ROS 2 packages are installed via `apt`, not `pip`
```bash
# Install ROS 2 first
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash

# Then install Python packages
pip install -r requirements.txt
```

#### "Could not find a version that satisfies the requirement speech-recognition"
**Problem**: Incorrect package name in requirements.txt
**Solution**: Use correct package name `SpeechRecognition`
```bash
# If you see this error, the requirements.txt has been updated
# Simply run pip install again:
pip install -r requirements.txt

# Or install manually:
pip install SpeechRecognition>=3.10.0
```

#### "Building wheel for pyaudio did not run successfully"
**Problem**: Missing system audio development libraries
**Solution**: System dependencies should be installed during main setup, but if you skipped them:
```bash
# Install system dependencies (now included in main installation steps)
sudo apt install \
    portaudio19-dev \
    python3-dev \
    build-essential \
    libasound2-dev \
    libffi-dev \
    libssl-dev \
    pkg-config

# Try pip install again
pip install -r requirements-pi.txt  # Use environment-specific file

# Alternative: Install PyAudio from system packages (if above fails)
sudo apt install python3-pyaudio
# Then remove pyaudio from requirements.txt and install the rest
```

#### "mock-package.py does not exist" or hardware package installation errors
**Problem**: Trying to install Raspberry Pi hardware packages on laptop/desktop
**Solution**: Use the correct requirements file for your environment
```bash
# For development on laptops/desktops:
pip install -r requirements-dev.txt

# For deployment on Raspberry Pi:
pip install -r requirements-pi.txt

# If you already tried the main requirements.txt and failed:
pip uninstall RPi.GPIO adafruit-circuitpython-pca9685 adafruit-circuitpython-motor adafruit-circuitpython-servokit gpiozero pigpio
pip install -r requirements-dev.txt
```

#### "GPG error: NO_PUBKEY F42ED6FBAB17C654" or "repository is not signed"
**Problem**: ROS 2 GPG key verification failure
**Solution**: Use our automated fix script or manual steps

**Quick Fix:**
```bash
# Run the automated GPG keyring fix
./scripts/fix_ros_keyring.sh
```

**Manual Fix:**
```bash
# Remove existing sources and try again
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Method 1: Use the updated keyring method
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Method 2: If Method 1 fails, use legacy apt-key method
# sudo apt install software-properties-common
# curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# sudo apt-add-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"

sudo apt update
sudo apt install ros-humble-desktop
```

#### "Package 'emu_vision' not found"
**Problem**: ROS workspace not built or sourced
**Solution**: Build and source the workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

#### Camera not detected
**Problem**: Camera drivers or permissions
**Solution**: Check camera connections and permissions
```bash
# Test cameras
libcamera-hello --list-cameras
sudo usermod -a -G video $USER  # Add user to video group
# Logout and login again
```

#### Hailo AI HAT+ not detected
**Problem**: HAI drivers not installed
**Solution**: Install Hailo drivers (requires Hailo Developer Zone access)
```bash
# Download from Hailo Developer Zone
# Install HailoRT package
sudo dpkg -i hailort-*.deb
hailortcli scan
```

#### Permission denied on GPIO/I2C
**Problem**: User not in required groups
**Solution**: Add user to hardware access groups
```bash
sudo usermod -a -G gpio,i2c,spi $USER
# Logout and login again
```

### Performance Issues

#### High CPU usage
- Reduce camera resolution in config files
- Adjust detection frequency
- Use Hailo AI acceleration

#### Memory errors
- Increase swap space
- Reduce batch sizes
- Monitor with `htop`

For more help, see `ros2_dependencies.md` or open an issue on GitHub.

## 🤝 Contributing

We welcome contributions to the Emu Droid project! Here's how to get involved:

### Development Process
1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### Contribution Areas
- 🔧 **Hardware Design**: Mechanical improvements, alternative components
- 🧠 **AI Models**: Custom model development, optimization
- 💻 **Software**: ROS node development, behavior programming
- 📚 **Documentation**: Tutorials, assembly guides, troubleshooting
- 🧪 **Testing**: Hardware validation, software testing, integration

### Code Standards
- **Python**: Follow PEP 8, use Black formatter
- **C++**: Follow ROS 2 conventions, use clang-format
- **Documentation**: Clear docstrings, inline comments for complex logic
- **Commits**: Descriptive messages, atomic changes

## 🎯 Roadmap to Mare Island Maker Faire 2026

| Phase | Timeline | Key Milestones |
|-------|----------|----------------|
| **Alpha Prototype** | Dec 2024 - May 2025 | Core functionality, basic demonstration |
| **Beta Development** | Jun 2025 - Dec 2025 | Optimization, reliability, multi-unit testing |
| **Production** | Jan 2026 - Aug 2026 | 5-unit build, field testing, demo preparation |
| **🎪 Maker Faire** | **September 2026** | **🏆 Live demonstration of 5 walking emu droids** |

**Current Status**: 🟢 Foundation Phase - Repository setup complete, hardware architecture defined

See [timeline.md](docs/timeline.md) for detailed project schedule and milestones.

## 📋 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### Open Source Philosophy
The Emu Droid project is committed to open-source principles:
- **🔓 Open Hardware**: All schematics, 3D models, and assembly instructions
- **💻 Open Software**: Complete source code, ROS packages, and AI models
- **📖 Open Documentation**: Build guides, tutorials, and educational materials
- **🤝 Open Community**: Welcoming contributions, collaboration, and forking

## 🆘 Support & Community

### Getting Help
- **📖 Documentation**: Start with README and [docs/](docs/) folder
- **🐛 Issues**: Report bugs via [GitHub Issues](https://github.com/makesingularity/mu-bot-project/issues)
- **💬 Discussions**: Join conversations in [GitHub Discussions](https://github.com/makesingularity/mu-bot-project/discussions)
- **📧 Contact**: Reach out to the team at dev@mubot.org

### Community Resources
- **🎥 Video Tutorials**: Assembly and development guides (coming soon)
- **🛠️ Workshop Materials**: Educational curriculum for schools/makerspaces
- **🏪 Parts Lists**: Curated supplier links and bulk purchase coordination
- **🤖 Showcase**: Share your emu droid builds and modifications

## 🏆 Acknowledgments

Special thanks to:
- **Hailo Technologies** for AI acceleration innovation
- **Raspberry Pi Foundation** for accessible computing platforms
- **ROS Community** for robotics software infrastructure
- **Maker Movement** for open-source hardware inspiration
- **Mare Island Maker Faire** for providing demonstration platform

---

**Built with ❤️ by the open-source robotics community**

*"Making companion robotics accessible, one emu at a time."*

🦆 **Happy Building!** 🤖
