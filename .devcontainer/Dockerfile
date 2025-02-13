ARG BASE_IMAGE=osrf/ros:jazzy-desktop
FROM $BASE_IMAGE

ARG DEBIAN_FRONTEND=noninteractive

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if id -u $USER_UID >/dev/null 2>&1; then \
        userdel -r $(getent passwd 1000 | cut -d: -f1); \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID --groups video -m $USERNAME;

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install essential packages
RUN sudo apt-get update && sudo apt-get install --no-install-recommends -y -o Dpkg::Options::="--force-overwrite" \
    # ros stuff
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz-sim-demos \
    ros-jazzy-ros-gz-bridge \
    # utils
    python3-pip \
    python-is-python3 \
    nano \
    less \
    xterm \
    libdrm-dev \
    mesa-utils &&\
    rm -rf /var/lib/apt/lists/*
    
# Bashrc scripts
COPY ./config/bashrc /tmp/bashrc

RUN cat /tmp/bashrc >> home/$USERNAME/.bashrc && \
    rm -f /tmp/bashr

# Make runtime directory
RUN mkdir -p /run/user/$USER_UID && \
    chown $USER_UID:$USER_GID /run/user/$USER_UID && \
    chmod 0700 /run/user/$USER_UID

# Change User
USER $USERNAME

# Rosdep update
RUN rosdep update