# Getting Started

This guide helps you set up and launch the Isaac ROS Workspace on your NVIDIA Jetson and development PC.

## Prerequisites
- Ubuntu 20.04 / 22.04 (Jetson and PC)
- ROS 2 Humble installed
- Docker and Docker Compose
- NVIDIA Jetson with JetPack SDK
- `colcon` and `rosdep`

## Clone the Workspace
```bash
git clone https://github.com/TNG-Blue/Isaac_ROS_WS.git
cd Isaac_ROS_WS
```

## Initialize the Environment
```bash
rosdep update
rosdep install --from-paths source --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---