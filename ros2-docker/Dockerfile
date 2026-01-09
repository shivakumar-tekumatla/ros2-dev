# Use official ROS 2 Humble base image
FROM ros:humble

# Build argument for username
ARG USER

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Update and install essential packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    vim \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 development tools
RUN apt-get update && apt-get install -y \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user for ROS development
RUN useradd -m -s /bin/bash ${USER} && \
    usermod -aG sudo ${USER} && \
    echo "${USER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/${USER}

# Set home directory
ENV HOME=/home/${USER}

# Set working directory
WORKDIR /home/${USER}/ros2-dev

# Create workspace structure
RUN mkdir -p /home/${USER}/ros2-dev && \
    chown -R ${USER}:${USER} /home/${USER}

# Switch to non-root user
USER ${USER}

# Source ROS 2 setup at container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f ~/ros2-dev/install/setup.bash ]; then source ~/ros2-dev/install/setup.bash; fi" >> ~/.bashrc && \
    echo "export PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc
