# Development Environment Instructions

This environment is set up for testing and developing ROS (Robot Operating System) code using GitHub Codespaces and GitHub MCP. Below is an overview of the environment and how to use it effectively.

## Environment Overview

1. **GitHub Codespaces**:
   - The development environment is hosted in GitHub Codespaces, providing a cloud-based workspace with pre-configured tools and dependencies for ROS development.
   - The home directory is set to `/workspaces`, ensuring all work is organized and accessible under `ubuntu@codespaces-4681e4:/workspaces$`.

2. **GitHub MCP**:
   - GitHub MCP (Model Context Protocol) is integrated into the environment for enhanced context management and dependency handling.

3. **Devcontainer Configuration**:
   - The `.devcontainer` folder contains the configuration files for the development container.
   - The container is based on the `osrf/ros:humble-desktop` image, pre-installed with ROS 2 Humble and additional tools for robotics development.

## Key Features

- **ROS 2 Development**:
  - The environment is optimized for developing and testing ROS 2 applications.
  - Tools like `rviz2`, `gazebo`, and `colcon` are pre-installed.

- **Graphical Interface**:
  - The XFCE desktop environment is accessible via noVNC, allowing you to interact with graphical tools directly in your browser.

- **Consistent Workspace**:
  - All files and projects are located under `/workspaces`, ensuring a consistent and organized directory structure.

## How to Use

1. **Access the Environment**:
   - Open the repository in GitHub Codespaces.
   - The terminal prompt should display: `ubuntu@codespaces-4681e4:/workspaces$`.

2. **Run ROS Commands**:
   - Use the terminal to execute ROS commands, such as:

     ```bash
     source /opt/ros/humble/setup.bash
     ros2 launch <package_name> <launch_file>
     ```

3. **Access the Desktop GUI**:
   - Open the **Ports** tab in VS Code and locate the forwarded port for noVNC.
   - Click the link to open the desktop GUI in your browser.

4. **Develop and Test**:
   - Use the terminal or GUI to develop and test ROS applications.
   - Launch simulation tools like Gazebo or visualization tools like RViz.

## Notes

- The home directory is explicitly set to `/workspaces` to ensure consistency across sessions.
- If you encounter any issues, refer to the `.devcontainer` configuration files for troubleshooting or customization.

Enjoy developing ROS code in this streamlined and powerful environment!
