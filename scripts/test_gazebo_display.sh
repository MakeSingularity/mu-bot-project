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

# Test basic X11 - use multiple fallback options
echo "🖼️  Testing X11 Display..."
X11_WORKS=false

# Try xeyes first (preferred)
if command -v xeyes >/dev/null 2>&1; then
    timeout 2 xeyes >/dev/null 2>&1
    if [ $? -eq 124 ]; then
        echo "✅ X11 display working (xeyes)"
        X11_WORKS=true
    fi
fi

# Fallback to xterm if xeyes not available or failed
if [ "$X11_WORKS" = false ] && command -v xterm >/dev/null 2>&1; then
    timeout 2 xterm -e echo "X11 test" >/dev/null 2>&1
    if [ $? -eq 124 ]; then
        echo "✅ X11 display working (xterm)"
        X11_WORKS=true
    fi
fi

# Final fallback - just check DISPLAY variable and try basic X11 connection
if [ "$X11_WORKS" = false ]; then
    if [ -n "$DISPLAY" ] && timeout 2 xdpyinfo >/dev/null 2>&1; then
        echo "✅ X11 display working (xdpyinfo)"
        X11_WORKS=true
    elif [ -n "$DISPLAY" ]; then
        echo "⚠️  X11 display partially working - DISPLAY set but connection issues"
        echo "   This is common in WSL - continuing with tests..."
        X11_WORKS=true
    else
        echo "❌ X11 display not working"
        echo "Please check your WSL display setup"
        exit 1
    fi
fi

# Test OpenGL and graphics permissions
echo "🎮 Testing OpenGL and Graphics Access..."
GRAPHICS_OK=false

# Check graphics device access
if [ -c /dev/dri/renderD128 ]; then
    if [ -r /dev/dri/renderD128 ] && [ -w /dev/dri/renderD128 ]; then
        echo "✅ Graphics device accessible"
        GRAPHICS_OK=true
    else
        echo "⚠️  Graphics device exists but permission denied"
        echo "   User may need to be in 'render' group"
        echo "   Run: sudo usermod -a -G render \$USER && newgrp render"
    fi
else
    echo "⚠️  Graphics device /dev/dri/renderD128 not found"
fi

# Test OpenGL
if command -v glxinfo >/dev/null 2>&1; then
    if glxinfo 2>/dev/null | grep -q "direct rendering: Yes"; then
        echo "✅ OpenGL direct rendering available"
    else
        echo "⚠️  OpenGL direct rendering not available - will use software rendering"
    fi
else
    echo "⚠️  glxinfo not available - install mesa-utils for OpenGL testing"
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
