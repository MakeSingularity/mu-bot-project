#!/bin/bash

# Laptop WSL VS Code Fix Script
# Run this on the laptop to fix the VS Code exec format error

echo "🔧 Fixing WSL VS Code Issue..."
echo ""

# Check current situation
echo "Current code command: $(which code)"
echo "Current wrapper content:"
if [ -f "/usr/local/bin/code" ]; then
    cat /usr/local/bin/code
else
    echo "No wrapper found at /usr/local/bin/code"
fi

echo ""
echo "Testing VS Code access methods..."

# Test code.cmd
echo "Testing code.cmd:"
if command -v code.cmd > /dev/null 2>&1; then
    echo "✅ code.cmd found"
    if code.cmd --version > /dev/null 2>&1; then
        echo "✅ code.cmd works"
    else
        echo "❌ code.cmd fails"
    fi
else
    echo "❌ code.cmd not found"
fi

# Test PowerShell method
echo ""
echo "Testing PowerShell method:"
if command -v powershell.exe > /dev/null 2>&1; then
    echo "✅ PowerShell available"
    if powershell.exe -c "Get-Command code" > /dev/null 2>&1; then
        echo "✅ VS Code accessible via PowerShell"
    else
        echo "❌ VS Code not found via PowerShell"
    fi
else
    echo "❌ PowerShell not available"
fi

echo ""
echo "🔄 Updating VS Code wrapper..."

# Replace the wrapper with our robust version
sudo cp /home/$USER/mu-bot/scripts/vscode_wsl_wrapper.sh /usr/local/bin/code
sudo chmod +x /usr/local/bin/code

echo "✅ Updated VS Code wrapper"

echo ""
echo "🧪 Testing new wrapper..."
if code --version > /dev/null 2>&1; then
    echo "✅ VS Code wrapper working!"
    echo "Version: $(code --version | head -1)"
else
    echo "❌ VS Code wrapper still failing"
    echo ""
    echo "Manual alternatives:"
    echo "  code.cmd .                    # Direct Windows command"
    echo "  powershell.exe -c 'code .'    # Via PowerShell"
    echo "  '/mnt/c/Program Files/Microsoft VS Code/Code.exe' .  # Direct path"
fi

echo ""
echo "🎯 Quick Test:"
echo "Try running: code --version"
echo "If that works, then: cd ~/mu-bot && code ."
