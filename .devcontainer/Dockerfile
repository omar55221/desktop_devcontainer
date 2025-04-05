FROM osrf/ros:humble-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
    RUN apt-get update && \
    apt-get install -y \
    ros-humble-twist-mux \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    ros-humble-slam-toolbox \
    ros-humble-controller-manager \
    ros-humble-xacro \
    ros-humble-gazebo-ros \
    ros-humble-rplidar-ros \
    ros-humble-usb-cam \
    ros-humble-image-transport-plugins \
    ros-humble-diff-drive-controller \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-rqt-image-view \
    ros-humble-rosbridge-suite \
    ros-humble-turtlebot3 \
    joystick \
    jstest-gtk \
    evtest \
    ros-humble-joint-state-broadcaster && \
    rm -rf /var/lib/apt/lists/*

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Set noninteractive mode for apt installs
ENV DEBIAN_FRONTEND=noninteractive

# Preseed debconf answers for keyboard and language configuration
# These answers set the keyboard layout to British and language to English.
RUN echo "keyboard-configuration keyboard-configuration/layout select British" | sudo debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/layoutcode string gb" | sudo debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/variant select English (UK)" | sudo debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/model select Generic 105-key (Intl) PC" | sudo debconf-set-selections && \
    echo "locales locales/default_environment_locale select en_US.UTF-8" | sudo debconf-set-selections

# Preseed the display manager selection to choose GDM3 as the default
RUN echo "gdm3 shared/default-x-display-manager select gdm3" | sudo debconf-set-selections

# Update all packages and install Git
RUN sudo apt update && sudo apt upgrade -y && \
    sudo apt install -y git

# Rosdep update and source ROS setup file
RUN rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

# Fix locale setup
RUN sudo apt-get update && \
    sudo apt-get install -y locales && \
    echo "en_US.UTF-8 UTF-8" | sudo tee /etc/locale.gen && \
    sudo locale-gen && \
    sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 LANGUAGE=en_US.UTF-8 || true

# Set locale environment variables explicitly
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install Ubuntu Desktop (GNOME), XFCE, and XFCE goodies
RUN sudo apt-get update && \
    sudo apt-get install -y ubuntu-desktop xfce4 xfce4-goodies

# Install VNC server and noVNC
RUN sudo apt-get install -y tigervnc-standalone-server tigervnc-common novnc websockify

# Configure VNC to start XFCE by default
RUN mkdir -p /home/ubuntu/.vnc && \
    echo '#!/bin/sh' > /home/ubuntu/.vnc/xstartup && \
    echo 'unset SESSION_MANAGER' >> /home/ubuntu/.vnc/xstartup && \
    echo 'unset DBUS_SESSION_BUS_ADDRESS' >> /home/ubuntu/.vnc/xstartup && \
    echo 'exec startxfce4' >> /home/ubuntu/.vnc/xstartup && \
    chmod +x /home/ubuntu/.vnc/xstartup && \
    sudo chown ubuntu:ubuntu /home/ubuntu/.vnc/xstartup

# Expose VNC and noVNC ports
EXPOSE 5901 6080

# Start VNC server and noVNC on container startup
CMD vncserver :1 -geometry 1920x1080 -depth 24 && \
    websockify --web=/usr/share/novnc/ 6080 localhost:5901