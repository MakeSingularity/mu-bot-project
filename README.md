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

### Prerequisites
- Ubuntu 22.04 LTS (native or WSL2)
- ROS 2 Humble Hawksbill
- Python 3.10 or higher
- Git and basic development tools

### 1. Repository Setup
```bash
# Clone the repository
git clone https://github.com/makesingularity/mu-bot-project.git
cd mu-bot-project

# Set up Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2. ROS 2 Environment Setup
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build ROS packages
cd src
colcon build --symlink-install
source install/setup.bash
```

### 3. Hardware Assembly (Optional)
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

### 4. Simulation (Gazebo)
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
├── src/                    # ROS 2 packages
│   ├── emu_control/        # Motor control and gait planning
│   ├── emu_vision/         # Computer vision and AI inference
│   └── emu_audio/          # Audio processing and TTS
├── sim/                    # Gazebo simulation
│   ├── worlds/             # Testing environments  
│   ├── urdf/               # Robot model definitions
│   └── launch/             # Simulation launch files
├── ai/                     # AI model pipeline
│   ├── hailo/              # Hailo model compilation
│   ├── pytorch/            # Custom model training
│   └── models/             # Compiled model binaries
├── hardware/               # Hardware documentation
│   ├── wiring_guide.md     # Complete wiring instructions
│   └── schematics.md       # Circuit diagrams and pinouts
├── docs/                   # Project documentation
│   ├── BOM.md              # Bill of materials with pricing
│   └── timeline.md         # Development timeline to 2026
├── tests/                  # Hardware and software tests
└── .vscode/                # Development environment config
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
