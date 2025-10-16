# Multi-Environment Network Configuration Guide

This guide explains how to configure networking between Desktop, Laptop, and Droid (Raspberry Pi) environments for seamless ROS 2 communication.

## 🌐 Network Topology

```
Desktop (192.168.1.100)     Laptop (192.168.1.101)     Droid (192.168.1.102)
├─ ROS Master               ├─ Remote Development       ├─ Hardware Control
├─ Simulation               ├─ Field Testing            ├─ Sensor Data
└─ AI Training              └─ Debugging                └─ Actuator Control
```

## 🔧 Configuration Steps

### 1. Ensure Network Connectivity

All devices should be on the same network (WiFi or Ethernet):

```bash
# Test connectivity between devices
ping 192.168.1.100  # Desktop
ping 192.168.1.101  # Laptop
ping 192.168.1.102  # Droid

# Check if ROS 2 ports are open (optional)
nmap -p 7400-7500 192.168.1.100
```

### 2. Configure ROS 2 Domain

Set the same domain ID on all systems:

```bash
# Add to ~/.bashrc on ALL systems
export ROS_DOMAIN_ID=42

# Apply immediately
source ~/.bashrc

# Verify
echo $ROS_DOMAIN_ID
```

### 3. Configure ROS 2 Discovery

#### Option A: Default Discovery (Recommended for small networks)
```bash
# No additional configuration needed
# ROS 2 will auto-discover nodes on the same domain
```

#### Option B: Discovery Server (For large networks or better performance)
```bash
# On Desktop (acting as discovery server)
export ROS_DISCOVERY_SERVER=192.168.1.100:11811
ros2 daemon stop
ros2 daemon start

# On Laptop and Droid (clients)
export ROS_DISCOVERY_SERVER=192.168.1.100:11811
ros2 daemon stop
ros2 daemon start

# Add to ~/.bashrc on client systems
echo "export ROS_DISCOVERY_SERVER=192.168.1.100:11811" >> ~/.bashrc
```

### 4. Test Multi-Environment Communication

#### Basic Node Discovery Test
```bash
# On Desktop - run a test publisher
ros2 run demo_nodes_cpp talker

# On Laptop - check if node is visible
ros2 node list
# Should show: /talker

# On Droid - listen to messages
ros2 topic echo /chatter
# Should receive messages from desktop
```

#### Emu-Specific Communication Test
```bash
# On Droid - start vision node
ros2 launch emu_vision emu_vision_launch.py

# On Laptop - monitor droid reports
ros2 topic echo /emu/report

# On Desktop - visualize in rviz
rviz2 -d config/emu_visualization.rviz
```

## 🛠️ Troubleshooting Network Issues

### Problem: Nodes not visible across environments
```bash
# Check ROS 2 domain
echo $ROS_DOMAIN_ID  # Should be same on all systems

# Check ROS 2 daemon status
ros2 daemon status

# Restart daemon if needed
ros2 daemon stop
ros2 daemon start
```

### Problem: High network latency
```bash
# Use FastRTPS with optimized settings
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Create cyclone config file
mkdir -p ~/.ros
cat > ~/.ros/cyclone_config.xml << EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDX xmlns="https://cyclonedx.org/schema" version="1.0">
  <Domain>
    <General>
      <NetworkInterfaceAddress>AUTO</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDX>
EOF

export CYCLONEDX_URI=file://$HOME/.ros/cyclone_config.xml
```

### Problem: Firewall blocking ROS 2
```bash
# Ubuntu/Debian firewall configuration
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
sudo ufw allow 11811/tcp  # Discovery server port

# Check firewall status
sudo ufw status
```

## 📊 Performance Monitoring

### Monitor ROS 2 Network Traffic
```bash
# Install monitoring tools
sudo apt install ros-humble-rqt-graph ros-humble-rqt-topic

# Visualize node graph
rqt_graph

# Monitor topic bandwidth
ros2 topic bw /emu/detection_image
ros2 topic hz /emu/report
```

### System Resource Monitoring
```bash
# Monitor CPU/memory on each system
htop

# Monitor network usage
iftop
nethogs

# ROS 2 specific monitoring
ros2 wtf  # Check ROS 2 configuration issues
```

## 🎯 Best Practices

### 1. Environment-Specific Optimizations

**Desktop (High Performance):**
- Full ROS 2 desktop installation
- High-quality simulation settings
- Parallel colcon builds

**Laptop (Portable):**
- Lightweight ROS 2 installation
- Battery-optimized settings
- Remote development focus

**Droid (Resource Constrained):**
- ROS 2 base installation only
- Limited parallel workers
- Hardware-optimized middleware

### 2. Development Workflow

1. **Develop** on Desktop with simulation
2. **Test** on Laptop with remote connection to Droid
3. **Deploy** final code to Droid hardware
4. **Monitor** all systems from Laptop during field testing

### 3. Security Considerations

```bash
# Use ROS 2 security if needed (advanced)
# Generate security certificates
ros2 security create_keystore demo_keystore
ros2 security create_enclave demo_keystore /emu_droid

# Enable security
export ROS_SECURITY_KEYSTORE=/path/to/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

## 📋 Quick Reference

### Essential Environment Variables
```bash
# Core ROS 2 configuration
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.1.100:11811  # Optional

# Performance tuning
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] [{name}] [{severity}]: {message}"
```

### Common Commands
```bash
# Network diagnostics
ros2 daemon status
ros2 node list
ros2 topic list
ros2 service list

# Performance monitoring
ros2 topic bw <topic_name>
ros2 topic hz <topic_name>
ros2 wtf

# System control
ros2 daemon stop/start
ros2 launch <package> <launch_file>
```
