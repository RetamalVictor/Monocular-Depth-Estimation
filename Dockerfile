# Monocular Depth Estimation - ROS 2 Docker Image
ARG ROS_DISTRO=humble

# =============================================================================
# CPU Build Target
# =============================================================================
FROM ros:${ROS_DISTRO} AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (CPU-only PyTorch)
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cpu \
    --extra-index-url https://pypi.org/simple \
    -r /tmp/requirements.txt

# Create workspace
WORKDIR /ros_ws

# Copy source code
COPY ros_ws/src /ros_ws/src

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# =============================================================================
# GPU Build Target (uses PyTorch with CUDA from pip)
# =============================================================================
FROM ros:${ROS_DISTRO} AS gpu-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (GPU with CUDA 12.1)
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --default-timeout=600 \
    --index-url https://download.pytorch.org/whl/cu121 \
    --extra-index-url https://pypi.org/simple \
    -r /tmp/requirements.txt

# Create workspace
WORKDIR /ros_ws

# Copy source code
COPY ros_ws/src /ros_ws/src

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
