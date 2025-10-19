#!/bin/bash

# Pi Post-Setup Quick Fix Script
# Run this if you encountered issues during setup_pi.sh

echo "🔧 Pi Setup Quick Fixes"
echo ""

# Fix 1: Group membership without reboot
echo "1. Fixing group membership for current session..."
echo "Current groups: $(groups)"

if groups | grep -q gpio; then
    echo "✅ Already in gpio group"
else
    echo "Adding to gpio group for current session..."
    newgrp gpio
fi

# Fix 2: VS Code project location
echo ""
echo "2. Setting up VS Code workspace..."
cd ~/mu-bot 2>/dev/null || cd /home/$USER/mu-bot 2>/dev/null || {
    echo "❌ mu-bot directory not found!"
    echo "Please navigate to your mu-bot project directory first."
    exit 1
}

echo "✅ In mu-bot directory: $(pwd)"

# Fix 3: Test VS Code and extensions
echo ""
echo "3. Testing VS Code installation..."

if command -v code > /dev/null 2>&1; then
    echo "✅ VS Code command found"

    # Test if VS Code can run
    if code --version > /dev/null 2>&1; then
        echo "✅ VS Code can run"

        # Check critical extensions
        echo "Checking extensions..."
        CRITICAL_EXTENSIONS=("github.copilot" "ms-python.python" "ms-vscode.cpptools")

        for ext in "${CRITICAL_EXTENSIONS[@]}"; do
            if code --list-extensions 2>/dev/null | grep -q "^$ext$"; then
                echo "✅ $ext installed"
            else
                echo "⚠️  $ext missing - installing..."
                code --install-extension "$ext" --force 2>/dev/null && echo "✅ Installed $ext" || echo "❌ Failed to install $ext"
            fi
        done
    else
        echo "⚠️  VS Code needs display environment"
        echo "If using SSH, try: ssh -X pi@your-pi-ip"
    fi
else
    echo "❌ VS Code not found - re-run setup_pi.sh"
fi

# Fix 4: Hardware interface test
echo ""
echo "4. Testing hardware interfaces..."

# Test GPIO access
if [ -w /dev/gpiomem ] 2>/dev/null; then
    echo "✅ GPIO access working"
else
    echo "⚠️  GPIO access issue - reboot may be needed"
fi

# Test I2C access
if command -v i2cdetect > /dev/null 2>&1; then
    echo "✅ I2C tools available"
    if [ -c /dev/i2c-1 ] 2>/dev/null; then
        echo "✅ I2C bus accessible"
    else
        echo "⚠️  I2C bus not accessible - check config.txt and reboot"
    fi
else
    echo "⚠️  I2C tools missing"
fi

echo ""
echo "🎯 Quick Actions:"
echo ""
echo "To open VS Code properly:"
echo "  cd ~/mu-bot && code ."
echo ""
echo "To install missing extensions manually:"
echo "  1. Open VS Code"
echo "  2. Ctrl+Shift+X (Extensions panel)"
echo "  3. Search for: GitHub Copilot, Python, C/C++"
echo "  4. Click Install for each"
echo ""
echo "To fix group membership permanently:"
echo "  sudo reboot"
echo ""
echo "To test hardware interfaces:"
echo "  i2cdetect -y 1    # Scan I2C bus"
echo "  groups            # Check group membership"
echo ""

if [ "$DISPLAY" ]; then
    echo "Display environment: $DISPLAY ✅"
else
    echo "No display environment - use SSH with -X flag for GUI apps"
fi

echo ""
echo "✨ Run this script again after reboot to verify everything works!"
