# Isaac Sim Setup Guide

## Prerequisites

Before installing Isaac Sim, ensure your system meets the following requirements:

### System Requirements
- **Operating System**: Windows 10/11 or Ubuntu 22.04 LTS
- **Processor**: 64-bit processor with 8+ cores
- **RAM**: 16GB+ (32GB recommended)
- **GPU**: NVIDIA GPU with 8GB+ VRAM (RTX 3060 or equivalent)
- **Storage**: 100GB+ free disk space
- **Network**: Internet connection for initial download and updates

### Software Requirements
- NVIDIA GPU drivers (version 510 or higher)
- ROS 2 Humble Hawksbill
- Docker (for containerized components)
- Git version control
- Python 3.8 or higher

## Installation Options

Isaac Sim can be installed using one of the following methods:

### Option 1: Omniverse Launcher (Recommended for beginners)
1. Visit the [NVIDIA Omniverse website](https://developer.nvidia.com/isaac-sim) to download the Omniverse Launcher
2. Create a free NVIDIA Developer account if you don't have one
3. Install the Omniverse Launcher application
4. Use the launcher to install Isaac Sim (free version)
5. Launch Isaac Sim from the Omniverse Launcher

### Option 2: Standalone Installation
1. Download Isaac Sim from the [NVIDIA Developer website](https://developer.nvidia.com/isaac-sim)
2. Extract the archive to a directory with sufficient space (e.g., `~/IsaacSim` on Linux or `C:\Users\[username]\IsaacSim` on Windows)
3. Run the installer and follow the setup instructions
4. Ensure your NVIDIA GPU drivers are up to date (version 510+)

## Initial Configuration

After installation, perform the following initial configuration steps:

### 1. Environment Setup
```bash
# Add Isaac Sim to your PATH (Linux example)
export ISAACSIM_PATH=~/IsaacSim
export PATH=$ISAACSIM_PATH:$PATH

# On Windows, add IsaacSim to your system PATH environment variable
```

### 2. ROS 2 Integration
To integrate Isaac Sim with ROS 2 Humble Hawksbill:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Set Isaac Sim ROS workspace
export ISAACSIM_ROS_WS=~/isaac_sim_ws
mkdir -p $ISAACSIM_ROS_WS/src
cd $ISAACSIM_ROS_WS

# Clone Isaac ROS packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git src/isaac_ros_common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark.git src/isaac_ros_benchmark
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_gxf.git src/isaac_ros_gxf

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

### 3. Verification
Verify your installation by launching Isaac Sim:

```bash
# On Linux
cd ~/IsaacSim
./isaac-sim.sh

# On Windows (using WSL2 or native)
cd C:\Users\[username]\IsaacSim
isaac-sim.bat
```

## Docker Container Setup (Alternative)

For a containerized approach, Isaac Sim can be run using Docker:

```bash
# Pull the Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:latest

# Run Isaac Sim container
docker run --gpus all -it --rm --network=host \
  --env "ACCEPT_EULA=Y" --env "USE_GAMEPAD=False" \
  --volume $(pwd):/workspace/shared \
  --volume ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  --volume ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  --volume ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia-omniverse/glcache:rw \
  --volume ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  --volume ~/docker/isaac-sim/config:/root/.nvidia-omniverse/config:rw \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
  --env="DISPLAY=${DISPLAY}" \
  --env="QT_X11_NO_MITSHM=1" \
  --privileged \
  --pid=host \
  nvcr.io/nvidia/isaac-sim:latest
```

## First Launch

When launching Isaac Sim for the first time:

1. **Accept the EULA** if prompted
2. **Configure graphics settings** for optimal performance
3. **Set up your workspace directory** for saving scenes and assets
4. **Verify GPU acceleration** is enabled in the settings

## Troubleshooting Common Issues

### GPU Not Detected
- Verify NVIDIA drivers are installed and up to date (version 510+)
- Check that your GPU meets the minimum requirements
- Ensure no other applications are using the GPU exclusively

### Performance Issues
- Reduce rendering quality in Isaac Sim settings
- Close unnecessary applications to free up system resources
- Verify sufficient RAM and VRAM availability

### ROS 2 Integration Issues
- Ensure ROS 2 Humble Hawksbill is properly installed
- Check that environment variables are set correctly
- Verify network configuration for multi-machine setups

## Next Steps

Once Isaac Sim is installed and configured, proceed to:
1. [Isaac Sim Prerequisites](./prerequisites.md) for detailed system requirements
2. [Isaac Sim Workflows](./workflows.md) for best practices
3. [Synthetic Data Generation](./synthetic-data-generation.md) to learn about creating training data