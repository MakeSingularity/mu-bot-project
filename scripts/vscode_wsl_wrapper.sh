#!/bin/bash

# VS Code WSL Wrapper Script - Path-aware version
# This script properly handles WSL to Windows path conversion

# Function to convert WSL path to Windows path
wsl_to_windows_path() {
    local wsl_path="$1"

    # If it's already a Windows path, return as-is
    if [[ "$wsl_path" =~ ^[A-Za-z]:\\ ]]; then
        echo "$wsl_path"
        return
    fi

    # Convert WSL path to Windows path
    if command -v wslpath > /dev/null 2>&1; then
        wslpath -w "$wsl_path" 2>/dev/null
    else
        # Fallback conversion for older WSL
        if [[ "$wsl_path" = /* ]]; then
            # Absolute WSL path
            if [[ "$wsl_path" =~ ^/mnt/([a-z])/(.*) ]]; then
                # Already in /mnt/c/ format
                drive="${BASH_REMATCH[1]^^}"
                path="${BASH_REMATCH[2]//\//\\}"
                echo "$drive:\\$path"
            else
                # WSL filesystem path - convert to Windows
                echo "$wsl_path" | sed 's|^/|\\\\wsl$\\Ubuntu\\|' | sed 's|/|\\|g'
            fi
        else
            # Relative path - make absolute first
            abs_path=$(realpath "$wsl_path" 2>/dev/null || echo "$(pwd)/$wsl_path")
            wsl_to_windows_path "$abs_path"
        fi
    fi
}

# Convert arguments to Windows paths where needed
converted_args=()
for arg in "$@"; do
    if [[ -e "$arg" ]] || [[ "$arg" == "." ]] || [[ "$arg" == ".."* ]] || [[ "$arg" =~ ^[./] ]]; then
        # This looks like a path, convert it
        converted_path=$(wsl_to_windows_path "$arg")
        converted_args+=("$converted_path")
    else
        # Not a path, keep as-is
        converted_args+=("$arg")
    fi
done

# Method 1: Try code.cmd (Windows command) with converted paths
if command -v code.cmd > /dev/null 2>&1; then
    exec code.cmd "${converted_args[@]}"
fi

# Method 2: Try PowerShell with proper path conversion
if command -v powershell.exe > /dev/null 2>&1; then
    # Build PowerShell command with quoted arguments
    ps_args=""
    for arg in "${converted_args[@]}"; do
        # Properly quote arguments for PowerShell
        if [[ "$arg" =~ [[:space:]] ]]; then
            ps_args="$ps_args '$arg'"
        else
            ps_args="$ps_args $arg"
        fi
    done

    exec powershell.exe -c "code$ps_args"
fi

# Method 3: Try direct Windows executable paths
for vscode_path in \
    "/mnt/c/Users/$USER/AppData/Local/Programs/Microsoft VS Code/Code.exe" \
    "/mnt/c/Program Files/Microsoft VS Code/Code.exe" \
    "/mnt/c/Program Files (x86)/Microsoft VS Code/Code.exe"; do

    if [ -f "$vscode_path" ]; then
        exec "$vscode_path" "${converted_args[@]}"
    fi
done

# If all methods fail, provide helpful error message
echo "❌ VS Code not found in WSL environment"
echo ""
echo "Debug info:"
echo "  Current directory: $(pwd)"
echo "  Windows path: $(wsl_to_windows_path "$(pwd)")"
echo "  Arguments: $*"
echo "  Converted args: ${converted_args[*]}"
echo ""
echo "Manual alternatives:"
echo "  code.cmd '$(wsl_to_windows_path "$(pwd)")'"
echo "  powershell.exe -c \"code '$(wsl_to_windows_path "$(pwd)")'\""
exit 1
