# Read the doc: https://huggingface.co/docs/hub/spaces-sdks-docker
# ROS 2 Humble Desktop Environment for Hugging Face Spaces

FROM osrf/ros:humble-desktop

# Set a non-interactive frontend for package installations
ENV DEBIAN_FRONTEND=noninteractive

# Create user with UID 1000 as required by Hugging Face Spaces
RUN useradd -m -u 1000 user
USER root

# Install system dependencies including desktop environment and Python
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo git locales python3 python3-pip \
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

# Set up VNC for the user
RUN mkdir -p /home/user/.vnc && \
    which vncpasswd && echo "vscode" | vncpasswd -f > /home/user/.vnc/passwd || \
    (echo "vncpasswd not found, using alternative" && \
     echo -n "vscode" | md5sum | cut -d' ' -f1 > /home/user/.vnc/passwd) && \
    chmod 600 /home/user/.vnc/passwd && \
    chown -R user:user /home/user/.vnc

# Configure VNC startup script
RUN echo '#!/bin/sh' > /home/user/.vnc/xstartup && \
    echo 'unset SESSION_MANAGER' >> /home/user/.vnc/xstartup && \
    echo 'unset DBUS_SESSION_BUS_ADDRESS' >> /home/user/.vnc/xstartup && \
    echo 'exec startxfce4' >> /home/user/.vnc/xstartup && \
    chmod +x /home/user/.vnc/xstartup && \
    chown user:user /home/user/.vnc/xstartup

# Give user sudo privileges
RUN echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user && \
    chmod 0440 /etc/sudoers.d/user

# Fix locale settings
RUN echo "en_US.UTF-8 UTF-8" | tee /etc/locale.gen && \
    locale-gen && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 LANGUAGE=en_US.UTF-8

ENV LANG=en_US.UTF-8 LANGUAGE=en_US.UTF-8 LC_ALL=en_US.UTF-8
ENV PATH="/home/user/.local/bin:$PATH"

# Switch to user and set working directory
USER user
WORKDIR /app

# Copy requirements and install Python dependencies
COPY --chown=user ./requirements.txt requirements.txt
RUN pip install --no-cache-dir --upgrade -r requirements.txt

# Copy application files
COPY --chown=user . /app

# Make scripts executable
RUN chmod +x start_desktop.sh fix_icons.sh

# Add ROS setup to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Expose the port that Hugging Face Spaces expects
EXPOSE 7860

# Start the Gradio application
CMD ["python3", "app.py"]
