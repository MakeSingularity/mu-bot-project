# Agent Development Instructions for Emu Droid Project

## 🎯 **PRIMARY GOAL**
Maintain cross-platform compatibility across Desktop, Laptop, and Raspberry Pi environments.

## ⚠️ **CRITICAL CONSTRAINTS - NEVER VIOLATE**

### Operating System Requirements:
- **Desktop**: Ubuntu 22.04 LTS (Jammy) - FIXED
- **Laptop**: Ubuntu 22.04 LTS (Jammy) - FIXED
- **Pi**: Raspberry Pi OS (Ubuntu 22.04 base) - FIXED
- **FORBIDDEN**: Debian 13+ (trixie), Ubuntu 24.04+, bleeding-edge distros

### Software Version Matrix:
- **Python**: 3.10.x ONLY (never 3.11+, never 3.9-)
- **ROS 2**: Humble ONLY (never Jazzy, Iron, Rolling)
- **Ubuntu Base**: 22.04 LTS ONLY (never 24.04+)

## 🛠️ **DEVELOPMENT WORKFLOW**

### Before ANY Code Changes:
1. **Ask**: "Will this work on Ubuntu 22.04 + Python 3.10 + ROS 2 Humble?"
2. **Check**: Does this require newer libraries/versions?
3. **Test**: Can this be verified on all 3 target devices?
4. **Document**: Add compatibility notes to changes

### Change Categories:

#### ✅ **SAFE CHANGES** (Always OK):
- Bug fixes within existing versions
- Documentation updates
- Configuration tweaks
- Package version patches (within same major.minor)

#### ⚠️ **RISKY CHANGES** (Require Extra Care):
- Adding new dependencies
- Changing build processes
- Updating package versions
- Hardware configuration changes

#### 🚫 **FORBIDDEN CHANGES** (Never Without User Approval):
- OS version upgrades
- Python version changes
- ROS distribution changes
- Major dependency overhauls

## 🔧 **TESTING PROTOCOL**

### For Small Changes:
1. Test on Desktop first
2. Verify compatibility with target versions
3. Document in commit message

### For Major Changes:
1. Create feature branch
2. Test on Desktop → Laptop → Pi (in order)
3. Verify with fresh install testing
4. Update COMPATIBILITY.md if needed

## 📝 **STANDARD RESPONSES**

### When User Requests Version Upgrade:
```
"Before upgrading to [X], let's check compatibility:
- Current: [Current versions]
- Proposed: [New versions]
- Risk: [Compatibility impact]
- Alternative: [Safer approach if available]

This change affects all 3 devices. Are you sure we want to risk breaking the working setup?"
```

### When Encountering Compatibility Issues:
```
"I found a compatibility issue:
- Problem: [Specific issue]
- Root cause: [Version mismatch/OS difference]
- Options:
  1. Revert to working versions
  2. Find compatible alternative
  3. Document limitation

Which approach do you prefer?"
```

## 🎯 **SUCCESS METRICS**
- ✅ All setup scripts work on fresh Ubuntu 22.04 installs
- ✅ Same commands work identically on all 3 devices
- ✅ No version conflicts between environments
- ✅ Clear documentation prevents future compatibility issues

## 💡 **REMEMBER**
**Stable > Latest**: Always prefer proven, stable versions over bleeding-edge releases
**Simple > Complex**: Favor simple solutions that work everywhere
**Test > Assume**: Verify compatibility, don't assume it
