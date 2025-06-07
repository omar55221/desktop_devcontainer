# Use the official ROS Humble desktop base image
FROM osrf/ros:humble-desktop

# Set a non-interactive frontend for package installations
ENV DEBIAN_FRONTEND=noninteractive

# Define arguments for user creation
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# OPTIMIZATION: Combine system setup, user creation, and sudo into one layer
RUN apt-get update && \
    apt-get install -y --no-install-recommends sudo git locales && \
    if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    # Fix locale settings
    echo "en_US.UTF-8 UTF-8" | tee /etc/locale.gen && \
    locale-gen && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 LANGUAGE=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8 LANGUAGE=en_US.UTF-8 LC_ALL=en_US.UTF-8

# OPTIMIZATION: Install all ROS and desktop packages in a single layer
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-twist-mux ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control \
    ros-humble-ros2-controllers ros-humble-joint-state-publisher-gui ros-humble-slam-toolbox \
    ros-humble-controller-manager ros-humble-xacro ros-humble-gazebo-ros \
    ros-humble-rplidar-ros ros-humble-usb-cam ros-humble-image-transport-plugins \
    ros-humble-diff-drive-controller ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy ros-humble-rqt-image-view ros-humble-rosbridge-suite \
    ros-humble-turtlebot3 ros-humble-joint-state-broadcaster \
    joystick jstest-gtk evtest \
    xfce4 xfce4-goodies tigervnc-standalone-server tigervnc-common novnc websockify \
    && rm -rf /var/lib/apt/lists/*

# Switch to the non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Add user to video group and source ROS setup
RUN sudo usermod --append --groups video $USERNAME && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# --- VNC Setup for Graphical Tools (Gazebo, RQT) ---
# CRITICAL FIX: Set a default VNC password ('vscode') non-interactively so the build doesn't hang
RUN mkdir -p /home/$USERNAME/.vnc && \
    echo "vscode" | vncpasswd -f > /home/$USERNAME/.vnc/passwd && \
    chmod 600 /home/$USERNAME/.vnc/passwd

# Configure VNC to start an XFCE desktop session
RUN echo '#!/bin/sh' > /home/$USERNAME/.vnc/xstartup && \
    echo 'unset SESSION_MANAGER' >> /home/$USERNAME/.vnc/xstartup && \
    echo 'unset DBUS_SESSION_BUS_ADDRESS' >> /home/$USERNAME/.vnc/xstartup && \
    echo 'exec startxfce4' >> /home/$USERNAME/.vnc/xstartup && \
    chmod +x /home/$USERNAME/.vnc/xstartup

# The command to run when the container starts.
# This starts the VNC server for the graphical desktop. The VS Code server
# will be started automatically by the Hugging Face Dev Mode infrastructure.
CMD vncserver :1 -geometry 1600x900 -depth 24 -rfbauth /home/ubuntu/.vnc/passwd && \
    websockify --web=/usr/share/novnc/ 6080 localhost:5901