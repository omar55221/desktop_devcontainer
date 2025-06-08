#!/bin/bash

# Desktop Icon Fix Script for VNC
echo "🔧 Refreshing desktop icons and themes..."

# Set better icon theme
DISPLAY=:1 xfconf-query -c xsettings -p /Net/IconThemeName -s "Adwaita"

# Set GTK theme  
DISPLAY=:1 xfconf-query -c xsettings -p /Net/ThemeName -s "Adwaita"

# Refresh icon caches
sudo gtk-update-icon-cache -f /usr/share/icons/hicolor/ 2>/dev/null
sudo gtk-update-icon-cache -f /usr/share/icons/Adwaita/ 2>/dev/null

# Restart panel to refresh icons
DISPLAY=:1 xfce4-panel --quit 2>/dev/null
sleep 1
DISPLAY=:1 xfce4-panel &

# Restart desktop to refresh background
DISPLAY=:1 xfdesktop --quit 2>/dev/null  
sleep 1
DISPLAY=:1 xfdesktop &

echo "✅ Desktop icons refreshed!"
echo "💡 If icons still don't appear correctly, try:"
echo "   - Right-click desktop → Applications → Settings → Appearance"
echo "   - Change icon theme to 'Adwaita' or 'Humanity'"
