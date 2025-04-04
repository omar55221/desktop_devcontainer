# ROS 2 Desktop in GitHub Codespaces

This repository provides a setup for running an Ubuntu desktop environment with ROS 2 Humble in a GitHub Codespace. The desktop environment is accessible via a web-based VNC viewer, allowing you to interact with a full graphical interface directly from your browser.

## Features
- Pre-installed ROS 2 Humble desktop environment.
- XFCE desktop environment for lightweight and responsive GUI.
- Accessible via noVNC in your browser.
- Simplified setup for consistent development environments.

## How to Use
1. **Open in VS Code**:
   - Clone this repository and open it in Visual Studio Code.
   - Reopen the project in the devcontainer when prompted.

2. **Access the Desktop GUI**:
   - Once the devcontainer is built, open the **Ports** tab in VS Code.
   - Locate the forwarded port for noVNC (default).
   - Click the link to open the desktop GUI in your browser.

3. **Set or Reset the Password** (Optional):  
   If the screen locks and asks for a password, you can set or reset the password for the user (e.g., `ubuntu`) by opening a terminal in the devcontainer and running:

   ```bash
   sudo passwd ubuntu
   ```

   This will ensure you don't get locked out of the desktop environment.

4. **Start Developing**:
   - Use the terminal in the desktop environment to run ROS 2 commands or launch applications like `rviz2`.

## Notes
- The setup is designed to work seamlessly in GitHub Codespaces.
- No additional configuration is required beyond opening the repository in a devcontainer.

Enjoy developing with ROS 2 in a fully graphical environment!
