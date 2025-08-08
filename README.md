# 🏎️ TurtleBot3 Patrol Navigation

A simple autonomous patrol system for TurtleBot3 using ROS2 Humble and Navigation2 stack. This project demonstrates basic autonomous navigation concepts with waypoint-based patrolling in Gazebo simulation.

## 🛡️ Features

- **Autonomous Patrol**: Robot follows predefined waypoints in a loop
- **Navigation2 Integration**: Uses Nav2 stack for path planning and obstacle avoidance  
- **Gazebo Simulation**: Full simulation environment support
- **RViz Visualization**: Real-time navigation visualization
- **Simple Configuration**: Easy-to-modify YAML waypoint configuration

## 💻 Prerequisites

- **Ubuntu 22.04 LTS**
- **ROS2 Humble** 
- **Gazebo Classic**
- **TurtleBot3 packages**
- **Navigation2 stack**

### Install Dependencies

```bash
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*

# Install Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install Gazebo and RViz
sudo apt install ros-humble-gazebo-* ros-humble-rviz2

# Install additional Nav2 dependencies
sudo apt install ros-humble-nav2-simple-commander
```

## 📁 Project Structure

```
turtlebot3_patrol_ws/
├── src/
│   └── patrol_navigation_project/
│       ├── package.xml
│       ├── setup.py
│       ├── resource/
│       │   └── patrol_navigation_project
│       ├── config/
│       │   ├── nav2_params.yaml
│       │   └── patrol_points.yaml
│       ├── launch/
│       │   ├── patrol_navigation.launch.py
│       │   └── patrol_gazebo.launch.py
│       ├── rviz/
│       │   └── patrol_config.rviz
│       └── patrol_navigation_project/
│           ├── __init__.py
│           ├── patrol_controller.py
│           └── waypoint_manager.py
└── README.md
```

## 🌐 Quick Start

### 1. Create Workspace and Package Structure

```bash
# Create workspace
mkdir -p ~/turtlebot3_patrol_ws/src
cd ~/turtlebot3_patrol_ws/src

# Create ROS2 package
ros2 pkg create --build-type ament_python patrol_navigation_project \
    --dependencies rclpy geometry_msgs nav2_simple_commander nav_msgs std_msgs ament_index_python

# Navigate to package directory
cd patrol_navigation_project

# Create directory structure
mkdir -p config launch rviz
mkdir -p resource

# Create resource file (required for ament_python packages)
touch resource/patrol_navigation_project
```

### 2. Create Required Files

```bash
# Create Python module files
touch patrol_navigation_project/__init__.py
touch patrol_navigation_project/patrol_controller.py
touch patrol_navigation_project/waypoint_manager.py

# Create configuration files
touch config/nav2_params.yaml
touch config/patrol_points.yaml

# Create launch files
touch launch/patrol_navigation.launch.py
touch launch/patrol_gazebo.launch.py

# Create RViz config (optional)
touch rviz/patrol_config.rviz
```

### 3. Environment Setup

```bash
# Add TurtleBot3 model to bashrc (recommended)
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
source ~/.bashrc

# Or export in each terminal session
export TURTLEBOT3_MODEL=waffle
```

### 4. Build the Package

```bash
cd ~/turtlebot3_patrol_ws

# Build the package
colcon build --packages-select patrol_navigation_project

# Source the workspace
source install/setup.bash
```

## 🎮 Running the System

### Option 1: Manual Launch (3 Terminals)

**Terminal 1: Launch Gazebo World**
```bash
export TURTLEBOT3_MODEL=waffle
source ~/turtlebot3_patrol_ws/install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2: Launch Navigation Stack**
```bash
export TURTLEBOT3_MODEL=waffle
source ~/turtlebot3_patrol_ws/install/setup.bash
ros2 launch patrol_navigation_project patrol_navigation.launch.py
```

**Terminal 3: Start Patrol**
```bash
export TURTLEBOT3_MODEL=waffle
source ~/turtlebot3_patrol_ws/install/setup.bash
ros2 run patrol_navigation_project patrol_controller
```

### Option 2: Complete Launch (1 Terminal)

```bash
export TURTLEBOT3_MODEL=waffle
source ~/turtlebot3_patrol_ws/install/setup.bash
ros2 launch patrol_navigation_project patrol_gazebo.launch.py
```

## ⚙️ Configuration

### Patrol Points (`config/patrol_points.yaml`)
Define your patrol route by modifying waypoints:

```yaml
patrol_points:
  - name: "point_1"
    x: 2.0
    y: 0.0
    z: 0.0
  - name: "point_2" 
    x: 2.0
    y: 2.0
    z: 0.0
  - name: "point_3" 
    x: 0.0
    y: 2.0
    z: 0.0
  - name: "point_4"
    x: 0.0
    y: 0.0
    z: 0.0
```

### Navigation Parameters (`config/nav2_params.yaml`)
Key parameters for robot behavior:
- **Velocity limits**: `max_vel_x`, `max_vel_theta` for speed control
- **Acceleration limits**: `acc_lim_x`, `acc_lim_theta` for smoothness
- **Goal tolerance**: `xy_goal_tolerance`, `yaw_goal_tolerance` for precision
- **Costmap settings**: Resolution and update frequencies

## 🔧 Key Components

### Core Files
- **`patrol_controller.py`**: Main patrol logic using Nav2 SimpleCommander
- **`waypoint_manager.py`**: YAML waypoint loader and sequence manager
- **`patrol_navigation.launch.py`**: Navigation stack launcher with RViz
- **`patrol_gazebo.launch.py`**: Complete simulation environment launcher

### Configuration Files
- **`patrol_points.yaml`**: Waypoint definitions
- **`nav2_params.yaml`**: Navigation behavior parameters
- **`package.xml`**: Package dependencies and metadata
- **`setup.py`**: Python package installation configuration

## 📊 System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Patrol         │    │  Nav2 Stack      │    │  Gazebo         │
│  Controller     ├────►  (Path Planning) ├────►  Simulation     │
│                 │    │  (Control)       │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         v                       v                       v
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Waypoint       │    │  RViz2           │    │  TurtleBot3     │
│  Manager        │    │  Visualization   │    │  Robot Model    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 📊 How It Works

1. **Initialization**: Waypoint Manager loads patrol points from YAML
2. **Navigation Setup**: Nav2 stack initializes with map and parameters  
3. **Patrol Execution**: Controller sends goals to Nav2 SimpleCommander
4. **Path Planning**: Nav2 computes optimal paths avoiding obstacles
5. **Motion Control**: Robot executes planned trajectory
6. **Loop Continuation**: System cycles through waypoints indefinitely

## 🐞 Troubleshooting

### Common Issues

**Robot doesn't move:**
```bash
# Check TurtleBot3 model
echo $TURTLEBOT3_MODEL

# Verify navigation topics
ros2 topic list | grep nav

# Check if goals are being published
ros2 topic echo /goal_pose
```

**Navigation fails:**
- Ensure initial pose is set in RViz (2D Pose Estimate)
- Check if waypoints are within map boundaries
- Verify costmaps are updating properly in RViz

**Build errors:**
```bash
# Clean build and rebuild
cd ~/turtlebot3_patrol_ws
rm -rf build install log
colcon build --packages-select patrol_navigation_project
```

**Launch file not found:**
```bash
# Make sure workspace is sourced
source ~/turtlebot3_patrol_ws/install/setup.bash

# Check if package is properly installed
ros2 pkg list | grep patrol_navigation_project
```

### Debug Commands

```bash
# Check node status
ros2 node list

# Monitor navigation status  
ros2 topic echo /navigation_result

# View transform tree
ros2 run tf2_tools view_frames
```
