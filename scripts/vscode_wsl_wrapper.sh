#!/bin/bash

# VS Code WSL Wrapper Script - Robust version
# This script tries multiple ways to access Windows VS Code from WSL

# Method 1: Try code.cmd (Windows command)
if command -v code.cmd > /dev/null 2>&1; then
    exec code.cmd "$@"
fi

# Method 2: Try direct Windows path
VSCODE_WIN_PATH="/mnt/c/Users/$USER/AppData/Local/Programs/Microsoft VS Code/Code.exe"
if [ -f "$VSCODE_WIN_PATH" ]; then
    exec "$VSCODE_WIN_PATH" "$@"
fi

# Method 3: Try common Windows paths
for user_dir in /mnt/c/Users/*; do
    if [ -d "$user_dir" ]; then
        vscode_path="$user_dir/AppData/Local/Programs/Microsoft VS Code/Code.exe"
        if [ -f "$vscode_path" ]; then
            exec "$vscode_path" "$@"
        fi
    fi
done

# Method 4: Try system-wide installation
VSCODE_SYSTEM_PATH="/mnt/c/Program Files/Microsoft VS Code/Code.exe"
if [ -f "$VSCODE_SYSTEM_PATH" ]; then
    exec "$VSCODE_SYSTEM_PATH" "$@"
fi

# Method 5: Fall back to PowerShell command
if command -v powershell.exe > /dev/null 2>&1; then
    exec powershell.exe -c "code $*"
fi

# If all methods fail, provide helpful error message
echo "❌ VS Code not found in WSL environment"
echo ""
echo "Please install VS Code on Windows first:"
echo "  1. Download from: https://code.visualstudio.com/"
echo "  2. Install with default options"
echo "  3. Restart WSL terminal"
echo "  4. Try 'code .' again"
echo ""
echo "Or try running directly:"
echo "  code.cmd ."
echo "  powershell.exe -c 'code .'"
exit 1
