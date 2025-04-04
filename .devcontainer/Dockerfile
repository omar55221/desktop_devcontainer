FROM osrf/ros:humble-desktop-full

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

# Install Ubuntu Desktop (GNOME), XFCE and XFCE goodies
RUN sudo apt update && \
    sudo apt install -y ubuntu-desktop xfce4 xfce4-goodies

# Configure VNC to start XFCE by default
RUN mkdir -p /home/ubuntu/.vnc && \
    echo '#!/bin/sh' > /home/ubuntu/.vnc/xstartup && \
    echo 'unset SESSION_MANAGER' >> /home/ubuntu/.vnc/xstartup && \
    echo 'unset DBUS_SESSION_BUS_ADDRESS' >> /home/ubuntu/.vnc/xstartup && \
    echo 'startxfce4 &' >> /home/ubuntu/.vnc/xstartup && \
    chmod +x /home/ubuntu/.vnc/xstartup
