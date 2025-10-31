# MOLA-SLAM

<div align="center">

**Real-time LiDAR SLAM using MOLA Framework for ROS2 Humble**

</div>

---

## 📋 Overview

Complete SLAM solution for:
- ✅ Real-time LiDAR odometry
- ✅ Map generation and visualization
- ✅ Multiple data export formats (TUM, PCD, MRPT)
- ✅ 3D LiDAR sensor support

---

## 🚀 Installation Guide

### Step 1: System Requirements

Before starting, ensure you have:
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS Distribution**: ROS2 Humble Hawksbill
- **Disk Space**: At least 10GB free for workspace and dependencies
- **Internet Connection**: Required for downloading packages

### Step 2: Install ROS2 Humble (if not already installed)

If you don't have ROS2 Humble installed:

```bash
# Add ROS2 repository
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistall/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y
```

### Step 3: Install System Dependencies

Install all required ROS2 packages and tools:

```bash
# Install essential ROS2 tools and dependencies
sudo apt update && sudo apt install -y \
    ros-humble-desktop \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    ros-humble-plotjuggler-ros \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rosbridge-suite \
    ros-humble-diagnostic-updater \
    python3-rosdep \
    python3-colcon-common-extensions
```

**What these packages do:**
- `ros-humble-desktop` - Core ROS2 functionality
- `ros-humble-rviz2` - 3D visualization tool
- `ros-humble-rqt*` - Qt-based GUI tools for debugging
- `ros-humble-plotjuggler-ros` - Real-time data plotting
- `ros-humble-tf2-tools` - Transform frame utilities
- `ros-humble-robot-state-publisher` - Publish robot state to TF
- `ros-humble-rosbridge-suite` - WebSocket interface for ROS
- `python3-rosdep` - Dependency management tool
- `python3-colcon-common-extensions` - Build system tools

### Step 4: Initialize rosdep

Rosdep helps install system dependencies for ROS packages:

```bash
# Initialize rosdep (skip if already initialized)
sudo rosdep init

# Update rosdep database
rosdep update
```

**Note**: If `rosdep init` fails with "already initialized", that's fine - just run `rosdep update`.

### Step 5: Create Workspace

Create a dedicated workspace for MOLA-SLAM:

```bash
# Create workspace directory structure
mkdir -p ~/mola_ws/src

# Navigate to source directory
cd ~/mola_ws/src
```

**Workspace Structure:**
```
~/mola_ws/
├── src/           # Source code (your packages go here)
├── build/         # Build artifacts (created during build)
├── install/       # Installed files (created during build)
└── log/           # Build logs (created during build)
```

### Step 6: Clone Repository

Clone the MOLA-SLAM repository into your workspace:

```bash
# Make sure you're in the src directory
cd ~/mola_ws/src

# Clone the repository
git clone https://github.com/Whan000/MOLA-SLAM.git

# Verify the clone
ls MOLA-SLAM/
```

### Step 7: Install Package Dependencies

Use rosdep to automatically install all dependencies required by the packages:

```bash
# Navigate to workspace root
cd ~/mola_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

**What this command does:**
- `--from-paths src` - Check all packages in src directory
- `--ignore-src` - Don't try to install packages that are being built
- `-r` - Continue even if some dependencies fail
- `-y` - Automatically answer yes to prompts

### Step 8: Build the Workspace

Source ROS2 and build all packages:

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build all packages in the workspace
colcon build

# Optional: Build with parallel jobs for faster compilation
# colcon build --parallel-workers 4
```

**Build Process:**
- This compiles all C++ code
- Processes Python packages
- Creates install directory with executables
- May take 5-15 minutes depending on your system

**Build Options:**
```bash
# Clean build (if you have issues)
colcon build --cmake-clean-cache

# Build specific package only
colcon build --packages-select [package_name]

# Build with verbose output (for debugging)
colcon build --event-handlers console_direct+
```

### Step 9: Source the Workspace

After building, source the workspace to use your packages:

```bash
# Source the workspace setup file
source ~/mola_ws/install/setup.bash

# Verify installation - list available packages
ros2 pkg list | grep mola
```

### Step 10: Automatic Sourcing (Recommended)

Add these lines to your `~/.bashrc` to automatically source on every terminal:

```bash
# Open bashrc in editor
nano ~/.bashrc

# Add these lines at the end:
source /opt/ros/humble/setup.bash
source ~/mola_ws/install/setup.bash

# Save and exit (Ctrl+X, then Y, then Enter)

# Reload bashrc
source ~/.bashrc
```

**Why do this?**
- No need to manually source in every new terminal
- Ensures ROS2 commands are always available
- Workspace packages are always accessible

### Step 11: Verify Installation

Test that everything is working:

```bash
# Check ROS2 environment
printenv | grep ROS

# List all ROS2 nodes (should see your packages)
ros2 pkg list

# Check if workspace is properly sourced
echo $ROS_PACKAGE_PATH
```

---

## ✅ Installation Complete!

Your MOLA-SLAM workspace is now ready to use. Proceed to the usage section to start running SLAM.

---

## 🎮 Basic Usage

### Launch SLAM
```bash
# Launch your SLAM node
ros2 launch [your_package] [your_launch_file].launch.py
```

### Open Visualization
```bash
# In a new terminal
rviz2
```

---

## 📊 Output Formats

| Format | Description |
|--------|-------------|
| **TUM** | Trajectory files for analysis & evaluation |
| **PCD** | Point cloud data for visualization |
| **MRPT Simplemap** | Native MOLA format for post-processing |
| **Metric Maps (.mm)** | Map files for visualization |

---

## 🔧 Common Issues

### Build Fails
```bash
# Clean and rebuild
cd ~/mola_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build
```

### Missing Dependencies
```bash
# Reinstall dependencies
cd ~/mola_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Package Not Found After Build
```bash
# Make sure workspace is sourced
source ~/mola_ws/install/setup.bash

# Or check if it's in your bashrc
cat ~/.bashrc | grep mola_ws
```

---

## 📦 Optional Packages

Install these if you need additional functionality:

**Navigation & Mapping**
```bash
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization
```

**Robot Control**
```bash
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers
```

**Simulation**
```bash
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control \
    ignition-fortress
```

---

## 📚 Useful Resources

- [MOLA Documentation](https://docs.mola-slam.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MRPT Library](https://www.mrpt.org/)

---

<div align="center">

**Made with ❤️ for the robotics community**

⭐ Star us on GitHub!

</div>