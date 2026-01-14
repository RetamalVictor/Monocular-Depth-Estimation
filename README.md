# Monocular Depth Estimation

Real-time monocular depth estimation using Intel's MiDaS model with ROS 2.

<p float="left">
  <img src="/assets/depth_plt.png?raw=true" width="400" />
  <img src="/assets/frame.png?raw=true" width="400" />
</p>

## Overview

This project provides a ROS 2-based pipeline for real-time depth estimation from a single camera. It uses Intel's MiDaS (Monocular Depth Estimation in the Wild) deep learning model to generate depth maps from RGB images.

### Architecture

```
Webcam (OpenCV)
    |
    v
[CameraComponent] ──> camera/image_raw ──> [MiDaSNode] ──> camera/depth_map ──> [DisplayComponent]
    (C++)                                    (Python)                              (C++)
```

### Packages

- **camera_pkg** (C++): Camera capture and depth map display components
- **depth_pkg** (Python): MiDaS depth estimation node

## Prerequisites

- Ubuntu 22.04 (or compatible)
- ROS 2 Humble or Iron
- Python 3.10+
- CUDA (optional, for GPU acceleration)

## Quick Start with Docker

The easiest way to run this project is with Docker.

### CPU Version

```bash
# Allow X11 forwarding for GUI
xhost +local:docker

# Build and run
docker compose up depth-estimation
```

### GPU Version (requires NVIDIA Docker)

```bash
# Install nvidia-container-toolkit if not already installed
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

xhost +local:docker
docker compose up depth-estimation-gpu
```

### Development Container

```bash
# Start interactive shell
docker compose run --rm dev

# Inside container: rebuild and test
colcon build --symlink-install
source install/setup.bash
ros2 launch depth_pkg depth_launch.py
```

### Docker Commands

```bash
# Build images
docker compose build

# Run with custom camera
docker compose run --rm depth-estimation \
    ros2 launch depth_pkg depth_launch.py camera_index:=2

# View logs
docker compose logs -f
```

## Native Installation

### 1. Install ROS 2 dependencies

```bash
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-rclcpp-components
```

### 2. Install Python dependencies

```bash
cd Monocular-Depth-Estimation
pip install -r requirements.txt
```

### 3. Build the workspace

```bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Run the full pipeline

```bash
# Source the workspace
source ros_ws/install/setup.bash

# Launch with default settings (camera index 0)
ros2 launch depth_pkg depth_launch.py

# Or specify a different camera
ros2 launch depth_pkg depth_launch.py camera_index:=2

# Use a different MiDaS model
ros2 launch depth_pkg depth_launch.py model_type:=DPT_Large
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_index` | `0` | Camera device index (e.g., `/dev/video0`) |
| `model_type` | `MiDaS_small` | MiDaS model variant |

### Available MiDaS models

| Model | Speed | Quality | GPU Memory |
|-------|-------|---------|------------|
| `MiDaS_small` | Fast | Good | ~500MB |
| `DPT_Hybrid` | Medium | Better | ~1GB |
| `DPT_Large` | Slow | Best | ~2GB |

### Run individual nodes

```bash
# Camera only
ros2 launch camera_pkg camera_launch.py

# Depth estimation only (requires camera topic)
ros2 run depth_pkg midas_node
```

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `camera/image_raw` | `sensor_msgs/Image` | Raw camera frames (BGR8) |
| `camera/depth_map` | `sensor_msgs/Image` | Depth map (mono8, normalized 0-255) |

## Parameters

### CameraComponent

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_index` | int | `0` | Camera device index |
| `topic_name` | string | `camera/image_raw` | Output topic name |
| `frame_width` | int | `640` | Frame width in pixels |
| `frame_height` | int | `480` | Frame height in pixels |

### MiDaSNode

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_type` | string | `MiDaS_small` | MiDaS model to use |

## Troubleshooting

### Camera not detected

Check available cameras:
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

### CUDA out of memory

Use a smaller model:
```bash
ros2 launch depth_pkg depth_launch.py model_type:=MiDaS_small
```

### Model download fails

The MiDaS model is downloaded on first run via `torch.hub`. Ensure you have internet connectivity. Models are cached in `~/.cache/torch/hub/`.

## License

MIT License
