#!/bin/bash

# ROS 2 Desktop Environment Startup Script
echo "🚀 Starting ROS 2 Desktop Environment..."

# Install D-Bus X11 support if not present (fixes XFCE session issues)
if ! which dbus-launch >/dev/null 2>&1; then
    echo "🔧 Installing D-Bus X11 support..."
    sudo apt update -qq && sudo apt install -y dbus-x11
fi

# Source ROS environment
source /opt/ros/humble/setup.bash

# Set display for GUI applications
export DISPLAY=:1

# Check if VNC server is already running
if pgrep -f "Xtigervnc :1" > /dev/null; then
    echo "✅ VNC server is already running"
else
    echo "🔧 Starting VNC server..."
    rm -f ~/.vnc/passwd  # Remove any existing password file
    vncserver :1 -geometry 1600x900 -depth 24 -SecurityTypes None -dpi 96
fi

# Check if websockify is already running
if pgrep -f "websockify.*6080" > /dev/null; then
    echo "✅ Web interface is already running"
else
    echo "🌐 Starting web interface..."
    websockify --web=/usr/share/novnc/ 6080 localhost:5901 &
    sleep 2
fi

echo ""
echo "🎉 Desktop environment is ready!"
echo ""

# Automatically fix icons after desktop starts
echo "🎨 Applying icon fixes..."
./fix_icons.sh

echo ""
echo "📱 Access options:"
echo "   • Browser: http://localhost:6080 (noVNC web interface - NO PASSWORD NEEDED)"
echo "   • VNC Client: localhost:5901 (direct VNC connection - NO PASSWORD NEEDED)"
echo ""
echo "🎮 Test GUI applications:"
echo "   export DISPLAY=:1"
echo "   rviz2 &"
echo "   gazebo &"
echo "   rqt &"
echo ""
echo "🔧 Fix icon display issues:"
echo "   ./fix_icons.sh"
echo ""
echo "💡 Tip: GUI apps can be launched from VS Code terminal or the desktop terminal"

# Keep container running
echo "🔄 Container ready - keeping services running..."
tail -f /dev/null
