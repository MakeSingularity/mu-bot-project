#!/bin/bash
# Test script for Gazebo Garden display compatibility in WSL
#
# This script tests various display configurations to find the best
# setup for running Gazebo Garden GUI in WSL environments.

echo "🧪 Testing Gazebo Garden Display Compatibility"
echo "=============================================="

# Check environment
echo "📋 Environment Check:"
echo "DISPLAY: $DISPLAY"
echo "WAYLAND_DISPLAY: $WAYLAND_DISPLAY"
echo "XDG_RUNTIME_DIR: $XDG_RUNTIME_DIR"
echo ""

# Test basic X11
echo "🖼️  Testing X11 Display..."
timeout 2 xeyes >/dev/null 2>&1
if [ $? -eq 124 ]; then
    echo "✅ X11 display working"
else
    echo "❌ X11 display not working"
    echo "Please check your WSL display setup"
    exit 1
fi

# Test OpenGL
echo "🎮 Testing OpenGL..."
if glxinfo | grep -q "direct rendering: Yes"; then
    echo "✅ OpenGL direct rendering available"
else
    echo "⚠️  OpenGL direct rendering not available - will use software rendering"
fi

# Test different Gazebo launch methods
echo ""
echo "🤖 Testing Gazebo Garden Compatibility..."

# Method 1: Software rendering
echo "Method 1: Software rendering..."
export LIBGL_ALWAYS_SOFTWARE=1
export QT_XCB_GL_INTEGRATION=none
if timeout 5 gz sim empty.sdf --headless >/dev/null 2>&1; then
    echo "✅ Gazebo headless mode working"
    HEADLESS_WORKS=true
else
    echo "❌ Gazebo headless mode failed"
    HEADLESS_WORKS=false
fi

# Method 2: Try GUI mode with different settings
echo "Method 2: GUI with compatibility settings..."
export LIBGL_ALWAYS_SOFTWARE=1
export QT_XCB_GL_INTEGRATION=none
export MESA_GL_VERSION_OVERRIDE=3.3

if timeout 3 gz sim empty.sdf >/dev/null 2>&1; then
    echo "✅ Gazebo GUI working with compatibility settings"
    GUI_WORKS=true
else
    echo "❌ Gazebo GUI failed - using headless mode only"
    GUI_WORKS=false
fi

# Recommendations
echo ""
echo "📝 Recommendations:"
if [ "$GUI_WORKS" = true ]; then
    echo "✅ Full GUI support available"
    echo "   Use: ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=true"
elif [ "$HEADLESS_WORKS" = true ]; then
    echo "⚠️  Headless mode only"
    echo "   Use: ros2 launch sim/launch/emu_gazebo_garden.launch.py gui:=false"
    echo "   Consider using RViz for visualization"
else
    echo "❌ Gazebo not working - please check installation"
fi

echo ""
echo "🔧 WSL Graphics Setup Tips:"
echo "1. Make sure WSLg is enabled: wsl --update"
echo "2. Restart WSL: wsl --shutdown && wsl"
echo "3. Install latest Windows Terminal"
echo "4. Consider using X410 or VcXsrv as alternative X server"

echo ""
echo "Test complete!"
