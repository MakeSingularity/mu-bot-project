# VS Code Copilot Terminal Troubleshooting Guide

This guide helps diagnose GitHub Copilot terminal integration issues across desktop, laptop, and Pi environments.

## 🔍 Diagnostic Commands

### 1. Check VS Code Installation & Extensions

**On Pi:**
```bash
# Check VS Code installation
which code
code --version

# List installed extensions
code --list-extensions | grep -E "(copilot|github)"

# Check if Copilot is properly installed
code --list-extensions | grep github.copilot
```

**On Laptop (WSL):**
```bash
# Check VS Code command accessibility
which code
code --version

# Check if Windows VS Code is accessible
which code.cmd
/mnt/c/Users/*/AppData/Local/Programs/Microsoft\ VS\ Code/Code.exe --version

# List extensions (may need Windows path)
code --list-extensions | grep copilot
```

### 2. Check GitHub Authentication

**All platforms:**
```bash
# Check if signed into GitHub in VS Code
# Run this from VS Code terminal (Ctrl+Shift+`)
code --log-extension github.copilot

# Or check authentication status
gh auth status  # if GitHub CLI is installed
```

### 3. Check Terminal Integration

**Test terminal access from VS Code:**
1. Open VS Code: `code .`
2. Open integrated terminal: `Ctrl+Shift+`` (backtick)
3. Try Copilot in terminal: Start typing a command and see if suggestions appear
4. Check Copilot status: Look for Copilot icon in VS Code status bar

### 4. Environment-Specific Diagnostics

**Pi-Specific Checks:**
```bash
# Check if running with proper display
echo $DISPLAY
echo $XDG_RUNTIME_DIR

# Check if desktop environment is available
echo $DESKTOP_SESSION
ps aux | grep -E "(gnome|xfce|lxde)"

# Check VS Code can access terminal
code --help | head -5
```

**WSL-Specific Checks:**
```bash
# Check WSL environment
cat /proc/version | grep microsoft
echo $WSL_DISTRO_NAME

# Check Windows integration
cmd.exe /c "code --version" 2>/dev/null
powershell.exe -c "Get-Command code" 2>/dev/null

# Check if WSL can communicate with Windows VS Code
code --version 2>&1 | grep -E "(error|exec format)"
```

## 🚨 Common Issues & Solutions

### Issue 1: Pi - "Copilot not working in terminal"
**Symptoms:** Copilot works in editor but not in integrated terminal
**Causes:**
- Extensions not installed (fixed by updated setup script)
- Display/desktop environment issues
- Permission problems

**Solutions:**
```bash
# 1. Ensure extensions are installed
./scripts/setup_pi.sh  # Re-run to install extensions

# 2. Check if VS Code can access terminal properly
export DISPLAY=:0  # If using local display
export DISPLAY=localhost:10.0  # If using SSH with X11 forwarding

# 3. Try running VS Code in different modes
code --verbose  # Check for errors
code --no-sandbox  # If permission issues
```

### Issue 2: WSL - "Exec format error"
**Symptoms:** VS Code fails to start or extensions don't work
**Causes:**
- Trying to execute Windows binary directly from WSL
- Path confusion between WSL and Windows VS Code
- Extension compatibility issues

**Solutions:**
```bash
# 1. Ensure proper VS Code wrapper is created
./scripts/setup_laptop.sh  # Re-run to fix WSL VS Code handling

# 2. Manually create wrapper if needed
sudo tee /usr/local/bin/code > /dev/null << 'EOF'
#!/bin/bash
exec code.cmd "$@"
EOF
sudo chmod +x /usr/local/bin/code

# 3. Use Windows VS Code directly
code.cmd .  # Instead of 'code .'
```

### Issue 3: GitHub Authentication Problems
**Symptoms:** Copilot installed but shows "not authenticated"
**Solutions:**
1. Open VS Code
2. `Ctrl+Shift+P` → "GitHub: Sign in"
3. Follow authentication flow
4. Restart VS Code after authentication

### Issue 4: Extension Installation Failures
**Symptoms:** Extensions fail to install during setup
**Manual Installation:**
1. Open VS Code
2. `Ctrl+Shift+X` (Extensions panel)
3. Search for "GitHub Copilot"
4. Click Install on:
   - GitHub Copilot
   - GitHub Copilot Chat

## 🔧 Advanced Diagnostics

### Check VS Code Logs
```bash
# VS Code extension logs
code --log trace

# Copilot-specific logs
ls ~/.vscode/extensions/github.copilot*
cat ~/.vscode/extensions/github.copilot*/extension.log
```

### Check System Resources
```bash
# Memory and CPU (Copilot needs resources)
free -h
top | head -10

# Network connectivity (Copilot needs internet)
ping github.com
curl -I https://api.github.com
```

### Test Different Scenarios
1. **Remote SSH to Pi from Desktop:** Does Copilot work when connecting remotely?
2. **Local Pi GUI:** Does Copilot work when using Pi directly with monitor?
3. **WSL vs Native Linux:** Compare behavior between WSL laptop and native desktop

## 📝 Collect Information for Further Help

If issues persist, collect this information:

```bash
# System information
uname -a
cat /etc/os-release
code --version
echo "Extensions:"
code --list-extensions | grep -E "(copilot|github|python|remote)"

# VS Code process information
ps aux | grep code
lsof -i | grep code  # Network connections

# Error logs
code --log-extension github.copilot 2>&1 | tail -20
```

## 🎯 Next Steps

1. Run diagnostics on each problematic system
2. Compare working (desktop) vs non-working (Pi/laptop) configurations
3. Apply specific fixes based on findings
4. Test Copilot functionality in both editor and terminal contexts

The updated Pi setup script now includes GitHub Copilot extensions, which should resolve the Pi-specific issues!
