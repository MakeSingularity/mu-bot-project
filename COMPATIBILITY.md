# Project Compatibility Matrix & Development Guidelines

## 🎯 **CRITICAL: Cross-Platform Compatibility Requirements**

> **⚠️ BEFORE ANY CHANGES**: All modifications must work across ALL three target environments. Test on each device before committing changes.

## 📋 **ACTUAL Hardware Constraints (Reality Check)**

| Device | OS Options | Python | ROS 2 | Reality |
|--------|----|---------|---------|---------|
| **Desktop Dev** | Ubuntu 22.04 LTS (Jammy) | Python 3.10.12 | ROS 2 Humble | ✅ Current Working |
| **Laptop Dev** | Ubuntu 22.04 LTS (Jammy) | Python 3.10.12 | ROS 2 Humble | ✅ Current Working |
| **Pi 5 Droid** | **Ubuntu 24.04+ MINIMUM** | Python 3.12+ | ROS 2 Jazzy | ❌ **INCOMPATIBLE** |

## � **CRITICAL DISCOVERY**
**Raspberry Pi 5 does NOT support Ubuntu 22.04** - minimum is Ubuntu 24.04.3 LTS

## 💡 **VIABLE SOLUTIONS**

### Option A: Upgrade All to Ubuntu 24.04
| Device | OS | Python | ROS 2 |
|--------|----|---------|---------|
| **Desktop** | Ubuntu 24.04 LTS | Python 3.12 | ROS 2 Jazzy |
| **Laptop** | Ubuntu 24.04 LTS | Python 3.12 | ROS 2 Jazzy |
| **Pi 5** | Ubuntu 24.04 LTS | Python 3.12 | ROS 2 Jazzy |
**Status**: ✅ True compatibility, requires Desktop/Laptop upgrades

### Option B: Mixed OS with Docker
| Device | Host OS | Container | ROS 2 |
|--------|----|---------|---------|
| **Desktop** | Ubuntu 22.04 | Docker (Ubuntu 24.04) | ROS 2 Jazzy |
| **Laptop** | Ubuntu 22.04 | Docker (Ubuntu 24.04) | ROS 2 Jazzy |
| **Pi 5** | Ubuntu 24.04 | Native or Docker | ROS 2 Jazzy |
**Status**: ✅ No host OS changes, consistent ROS environment

### Option C: Raspberry Pi OS + Source Build
| Device | OS | Python | ROS 2 |
|--------|----|---------|---------|
| **Desktop** | Ubuntu 22.04 | Python 3.10 | ROS 2 Humble |
| **Laptop** | Ubuntu 22.04 | Python 3.10 | ROS 2 Humble |
| **Pi 5** | Raspberry Pi OS | Python 3.11+ | ROS 2 Humble (source) |
**Status**: ⚠️ Different OS, complex Pi build process

## 🛠️ **Development Rules**

### Before Making Changes:
1. **Check Python version**: Must be 3.10.x across all devices
2. **Check OS compatibility**: Must support ROS 2 Humble packages
3. **Test on all 3 devices**: Desktop → Laptop → Pi
4. **Version lock dependencies**: Pin to specific versions in requirements files

### Pi Setup Requirements:
- **OS**: Raspberry Pi OS 64-bit (based on Ubuntu 22.04/Debian 11)
- **NOT**: Debian 13 (trixie) or other bleeding-edge distros
- **Python**: 3.10.x (check with `python3 --version`)
- **ROS**: 2 Humble only

### Change Management:
- **Small changes**: Test on Desktop first, then others
- **Major changes**: Create feature branch, test all environments
- **Version upgrades**: Research compatibility BEFORE implementing
- **Dependencies**: Always check cross-platform availability

## 🔧 **Quick Compatibility Check Commands**

```bash
# Run on each device before major changes
echo "=== Compatibility Check ==="
echo "OS: $(lsb_release -d)"
echo "Python: $(python3 --version)"
echo "ROS: $ROS_DISTRO"
ros2 --version
echo "=========================="
```

## 📝 **Lessons Learned**
- **Stable > Bleeding Edge**: Use proven, stable versions over latest
- **Test Early, Test Often**: Compatibility issues compound quickly
- **Document Constraints**: Make limitations visible and enforceable
- **Question Upgrades**: "Does this upgrade provide value > compatibility risk?"

## 🎯 **Success Criteria**
✅ All setup scripts work on fresh installs
✅ Same ROS commands work on all 3 devices
✅ Same Python packages install everywhere
✅ Development workflow identical across environments
