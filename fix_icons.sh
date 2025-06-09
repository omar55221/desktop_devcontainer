#!/bin/bash

# Desktop Icon Fix Script for VNC
echo "🔧 Refreshing desktop icons and themes..."

# Ensure we're using the correct display
export DISPLAY=:1

# Kill any existing panel/desktop processes that might be zombie
pkill -f xfce4-panel 2>/dev/null || true
pkill -f xfdesktop 2>/dev/null || true
sleep 2

# Set better icon and GTK themes using xfconf
xfconf-query -c xsettings -p /Net/IconThemeName -s "gnome" 2>/dev/null || echo "Warning: Could not set icon theme"
xfconf-query -c xsettings -p /Net/ThemeName -s "Adwaita" 2>/dev/null || echo "Warning: Could not set GTK theme"

# Set window manager theme
xfconf-query -c xfwm4 -p /general/theme -s "Default" 2>/dev/null || echo "Warning: Could not set WM theme"

# Update icon caches
echo "🔄 Updating icon caches..."
sudo gtk-update-icon-cache -f /usr/share/icons/hicolor/ 2>/dev/null || true
sudo gtk-update-icon-cache -f /usr/share/icons/Adwaita/ 2>/dev/null || true  
sudo gtk-update-icon-cache -f /usr/share/icons/gnome/ 2>/dev/null || true

# Clear any panel configuration that might be corrupted
rm -f ~/.config/xfce4/panel/panels.xml 2>/dev/null || true

# Start desktop and panel with proper environment
echo "🖥️ Starting desktop components..."
nohup xfdesktop --disable-wm-check > /dev/null 2>&1 &
sleep 2
nohup xfce4-panel --disable-wm-check > /dev/null 2>&1 &

# Wait a moment for things to settle
sleep 3

echo "✅ Desktop icons refreshed!"
echo "💡 If icons still don't appear correctly, try:"
echo "   - Right-click desktop → Applications → Settings → Appearance"
echo "   - Change icon theme to 'gnome' or 'Adwaita'"
echo "   - Change window theme to 'Default'"
