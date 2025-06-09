---
title: ROS 2 Humble Desktop
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: docker
app_port: 7860
pinned: false
license: mit
---

# ROS 2 Desktop Environment

A complete ROS 2 Humble development environment with desktop GUI support running in VS Code DevContainers and Hugging Face Spaces.

This repository provides a setup for running an Ubuntu desktop environment with ROS 2 Humble in both GitHub Codespaces and Hugging Face Spaces. The desktop environment is accessible via a web-based VNC viewer, allowing you to interact with a full graphical interface directly from your browser.

## Features
- Pre-installed ROS 2 Humble desktop environment.
- XFCE desktop environment for lightweight and responsive GUI.
- Accessible via noVNC in your browser.
- Simplified setup for consistent development environments.
- Support for both GitHub Codespaces and Hugging Face Spaces.

## How to Use

### 🔧 **GitHub Codespaces / VS Code DevContainers**
1. **Open in VS Code**:
   - Clone this repository and open it in Visual Studio Code.
   - Reopen the project in the devcontainer when prompted.

2. **Access the Desktop GUI**:
   - Once the devcontainer is built, open the **Ports** tab in VS Code.
   - Locate the forwarded port for noVNC (default: 6080).
   - Click the link to open the desktop GUI in your browser.

### 🚀 **Hugging Face Spaces**
1. **Start Desktop Environment**:
   - Use the Gradio interface to start the desktop environment.
   - Click "🚀 Start Desktop Environment" button.

2. **Access Desktop**:
   - Navigate to the port 6080 tab or the noVNC URL.
   - Access the full Ubuntu desktop with ROS 2 tools.

3. **Available Tools**:

   ```bash
   rviz2              # Launch RViz2
   gazebo             # Launch Gazebo simulator  
   rqt                # Launch RQT tools
   ./start_desktop.sh # Start/restart desktop
   ./fix_icons.sh     # Fix desktop icons/themes
   ```

4. **Set or Reset the Password** (Optional):
   If the screen locks and asks for a password, you can set or reset the password for the user (e.g., `ubuntu`) by opening a terminal in the devcontainer and running:

   ```bash
   sudo passwd ubuntu
   ```

   This will ensure you don't get locked out of the desktop environment.

5. **Start Developing**:
   - Use the terminal in the desktop environment to run ROS 2 commands or launch applications like `rviz2`.

## Notes

- The setup is designed to work seamlessly in GitHub Codespaces.
- No additional configuration is required beyond opening the repository in a devcontainer.

Enjoy developing with ROS 2 in a fully graphical environment!
