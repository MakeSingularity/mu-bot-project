# Hardware Wiring Guide for Emu Droid Companion Robot

## Overview
This document provides detailed wiring instructions for connecting the emu droid hardware components to the Raspberry Pi 5 with stacked HATs.

## Hardware Stack Configuration

### Layer 1: Raspberry Pi 5 (16GB RAM)
- **GPIO Header**: 40-pin GPIO for HAT connections
- **CSI Ports**: 2x MIPI CSI for stereo cameras
- **Power**: USB-C 5V/5A power supply
- **Storage**: 64GB+ microSD card (Class 10 or better)

### Layer 2: Hailo AI HAT+ (26 TOPS)
- **Connection**: Stacks directly on Pi GPIO header
- **Power**: Powered through GPIO 5V rail
- **Interface**: PCIe communication via GPIO
- **Cooling**: Ensure adequate heatsink/fan

### Layer 3: Waveshare WM8960 Audio HAT
- **Connection**: Stacks on Hailo HAT GPIO passthrough
- **I2S Interface**: Uses GPIO 18-21 for audio
- **Features**: Stereo input/output, built-in amplifier

### Layer 4: PCA9685 16-Channel PWM HAT
- **Connection**: Stacks on WM8960 GPIO passthrough  
- **I2C Interface**: Uses GPIO 2 (SDA) and GPIO 3 (SCL)
- **Address**: Default 0x40 (configurable via jumpers)
- **Power**: External 5V/10A supply for servo power

## Detailed Wiring Connections

### GPIO Pin Assignment
```
GPIO Pin | Function           | Component
---------|-------------------|------------------
GPIO 2   | I2C SDA           | PCA9685 HAT
GPIO 3   | I2C SCL           | PCA9685 HAT  
GPIO 18  | I2S BCK           | WM8960 Audio HAT
GPIO 19  | I2S LRCK          | WM8960 Audio HAT
GPIO 20  | I2S DIN           | WM8960 Audio HAT
GPIO 21  | I2S DOUT          | WM8960 Audio HAT
CSI-0    | Camera Interface  | Left Arducam 5MP
CSI-1    | Camera Interface  | Right Arducam 5MP
```

### Camera Connections

#### Left Camera (CSI-0 Port)
- **Model**: Arducam 5MP Motorized Focus (B0344)
- **Sensor**: OV5647, 1080p@30fps capability
- **Cable**: 15cm flex cable to CSI-0 port
- **Focus Motor**: I2C control via camera module
- **Frame ID**: `left_camera_optical_frame`

#### Right Camera (CSI-1 Port)  
- **Model**: Arducam 5MP Motorized Focus (B0344)
- **Sensor**: OV5647, 1080p@30fps capability
- **Cable**: 20cm flex cable to CSI-1 port (longer for positioning)
- **Focus Motor**: I2C control via camera module
- **Frame ID**: `right_camera_optical_frame`

### Servo Connections (PCA9685)

#### Eye Platform Servos (Stewart Platform)
Each eye has 3 servos for 3-DOF movement:

**Left Eye Stewart Platform:**
- Servo 1 (Base): PWM Channel 0
- Servo 2 (Mid): PWM Channel 1  
- Servo 3 (Top): PWM Channel 2

**Right Eye Stewart Platform:**
- Servo 1 (Base): PWM Channel 3
- Servo 2 (Mid): PWM Channel 4
- Servo 3 (Top): PWM Channel 5

#### Neck Cable Servos
Cable-driven neck control using 6 servos:
- Neck Servo 1: PWM Channel 6
- Neck Servo 2: PWM Channel 7
- Neck Servo 3: PWM Channel 8
- Neck Servo 4: PWM Channel 9
- Neck Servo 5: PWM Channel 10
- Neck Servo 6: PWM Channel 11

#### Reserved Channels
- Channels 12-15: Reserved for future expansion (head tilt, etc.)

### Power Distribution

#### Primary Power (Raspberry Pi + HATs)
- **Input**: 5V/5A USB-C power supply
- **Distribution**: Powers Pi, Hailo AI HAT, WM8960 via GPIO
- **Protection**: TVS diodes on power rails

#### Servo Power (External)
- **Input**: 5V/10A switching power supply
- **Connection**: Connect to PCA9685 V+ and GND terminals
- **Distribution**: Powers all 12 SG90 servos
- **Protection**: 10A fuse, reverse polarity protection

#### Power Consumption Estimates
```
Component              | Idle Power | Active Power
-----------------------|------------|-------------
Raspberry Pi 5         | 3.0W       | 8.0W
Hailo AI HAT+         | 2.0W       | 15.0W
WM8960 Audio HAT      | 0.5W       | 1.5W
PCA9685 PWM HAT       | 0.2W       | 0.5W
2x Arducam 5MP        | 1.0W       | 2.0W
12x SG90 Servos       | 6.0W       | 36.0W
-----------------------|------------|-------------
Total System          | 12.7W      | 63.0W
```

## Assembly Instructions

### Step 1: Prepare Raspberry Pi 5
1. Install 64GB microSD with Ubuntu 22.04 + ROS 2 Humble
2. Attach GPIO stacking header (if not pre-installed)
3. Install heatsink on Pi CPU
4. Test basic Pi functionality

### Step 2: Install Hailo AI HAT+
1. Carefully align HAT with Pi GPIO header
2. Press down firmly to ensure all pins connect
3. Secure with provided standoffs and screws
4. Attach heatsink/fan to Hailo chip
5. Test Hailo detection: `hailortcli scan`

### Step 3: Install WM8960 Audio HAT
1. Stack on top of Hailo HAT GPIO passthrough
2. Ensure I2S pins (18-21) make good contact
3. Secure with standoffs
4. Test audio: `arecord -l` and `aplay -l`

### Step 4: Install PCA9685 PWM HAT
1. Stack on top of WM8960 HAT
2. Verify I2C pins (2,3) connection
3. Set I2C address jumpers if needed (default 0x40)
4. Connect external 5V servo power supply
5. Test PWM: `i2cdetect -y 1`

### Step 5: Install Cameras
1. Connect left camera to CSI-0 with 15cm cable
2. Connect right camera to CSI-1 with 20cm cable
3. Mount cameras in eye assemblies with proper baseline (12cm)
4. Test cameras: `libcamera-hello --camera 0` and `--camera 1`

### Step 6: Connect Servos
1. Connect eye platform servos to channels 0-5
2. Connect neck servos to channels 6-11
3. Verify servo power connections (red=5V, brown=GND, orange=signal)
4. Test servo movement: `python3 servo_test.py`

### Step 7: Final Integration
1. Mount all assemblies in emu frame
2. Route cables cleanly with cable management
3. Verify all connections before powering on
4. Test full system integration

## Troubleshooting

### Common Issues

**Camera not detected:**
- Check CSI cable connection and orientation
- Verify camera power (3.3V on CSI connector)
- Test with `libcamera-hello`

**Servo not responding:**
- Check PWM signal with oscilloscope
- Verify servo power (5V on red wire)
- Test PCA9685 I2C communication

**Audio HAT not working:**
- Check I2S pin connections (18-21)
- Verify WM8960 I2C address (0x1a)
- Test with `speaker-test`

**Hailo AI not responding:**
- Check HAT seating on GPIO header
- Verify Hailo driver installation
- Test with `hailortcli scan`

### Test Commands
```bash
# Test cameras
libcamera-hello --camera 0 --timeout 5000
libcamera-hello --camera 1 --timeout 5000

# Test I2C devices
i2cdetect -y 1

# Test Hailo AI
hailortcli scan
hailortcli benchmark --model yolov8n.hef

# Test audio
arecord -f cd -t wav -d 5 test.wav
aplay test.wav

# Test servos
python3 -c "
import board
import busio
from adafruit_pca9685 import PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50
pca.channels[0].duty_cycle = 0x7FFF  # Move servo 0
"
```

## Safety Considerations

1. **Power Safety**: Always disconnect power before wiring changes
2. **ESD Protection**: Use anti-static wrist strap when handling HATs
3. **Heat Management**: Ensure adequate cooling for Pi and Hailo HAT
4. **Mechanical Safety**: Secure all connections against vibration
5. **Servo Limits**: Implement software limits to prevent mechanical damage

## Bill of Materials Reference

See `docs/BOM.md` for complete parts list with supplier information and current pricing.