ARG BASE_IMAGE=osrf/ros:jazzy-desktop-full
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
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME;

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install essential packages
RUN apt-get update && apt-get install --no-install-recommends -y -o Dpkg::Options::="--force-overwrite" \
    ros-jazzy-ros-gz \
    ros-jazzy-joint-state-publisher-gui \
    python3-pip \
    python-is-python3 \
    nano \
    less \
    xterm && \
    rm -rf /var/lib/apt/lists/*

# Copy config files
COPY ./config/bashrc /tmp/bashrc

#  Config container
RUN cat /tmp/bashrc >> home/$USERNAME/.bashrc && \
    rm -f /tmp/bashr

# Make /run/user/1000 directory (idk why we have to do this sometimes)
RUN mkdir -p /run/user/$USER_UID && \
    chown $USER_UID:$USER_GID /run/user/$USER_UID && \
    chmod 0700 /run/user/$USER_UID

# Change user from root
USER $USERNAME

# Rosdep update
RUN rosdep update


