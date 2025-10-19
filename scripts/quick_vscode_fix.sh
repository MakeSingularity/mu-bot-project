#!/bin/bash

# Quick WSL VS Code Fix - Simple Working Solution
# Run this on the laptop for an immediate working fix

echo "🚀 Quick VS Code Fix for WSL"
echo ""

echo "Since powershell.exe -c 'code .' works but opens wrong directory,"
echo "let's create a proper working solution..."
echo ""

# Method 1: Simple working wrapper that uses wslpath
echo "Creating working VS Code wrapper..."

sudo tee /usr/local/bin/code > /dev/null << 'EOF'
#!/bin/bash

# Convert current directory or arguments to Windows paths
convert_args=()
for arg in "$@"; do
    if [[ "$arg" == "." ]]; then
        # Convert current directory
        if command -v wslpath > /dev/null 2>&1; then
            convert_args+=($(wslpath -w "$(pwd)"))
        else
            # Fallback if wslpath not available
            convert_args+=("$(pwd | sed 's|^/mnt/\([a-z]\)/|\U\1:/|' | sed 's|/|\\|g')")
        fi
    elif [[ -e "$arg" ]] || [[ "$arg" =~ ^[./] ]]; then
        # This is a path, convert it
        if command -v wslpath > /dev/null 2>&1; then
            convert_args+=($(wslpath -w "$arg"))
        else
            # Fallback conversion
            abs_path=$(realpath "$arg" 2>/dev/null || echo "$(pwd)/$arg")
            convert_args+=("$(echo "$abs_path" | sed 's|^/mnt/\([a-z]\)/|\U\1:/|' | sed 's|/|\\|g')")
        fi
    else
        # Not a path, keep as-is
        convert_args+=("$arg")
    fi
done

# Use PowerShell to launch VS Code with properly converted paths
exec powershell.exe -c "code $(printf "'%s' " "${convert_args[@]}")"
EOF

sudo chmod +x /usr/local/bin/code

echo "✅ Created working VS Code wrapper"
echo ""

# Test the fix
echo "🧪 Testing the fix..."
echo ""

echo "Testing: code --version"
if code --version > /dev/null 2>&1; then
    echo "✅ VS Code version check works"
    code --version | head -1
else
    echo "❌ Version check failed"
fi

echo ""
echo "Current directory: $(pwd)"
echo "Windows equivalent: $(wslpath -w "$(pwd)" 2>/dev/null || echo "wslpath not available")"

echo ""
echo "🎯 Now try:"
echo "  cd ~/mu-bot"
echo "  code ."
echo ""
echo "This should open VS Code in the correct mu-bot directory!"
