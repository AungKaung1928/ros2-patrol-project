# ROS 2 Patrol Navigation Project 🗺️⚡

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white" alt="ROS2 Humble">
  <img src="https://img.shields.io/badge/Python-3.8+-yellow?style=for-the-badge&logo=python&logoColor=white" alt="Python">
  <img src="https://img.shields.io/badge/Gazebo-Simulation-green?style=for-the-badge&logo=gazebo&logoColor=white" alt="Gazebo">
  <img src="https://img.shields.io/badge/RViz-Visualization-orange?style=for-the-badge&logo=rviz&logoColor=white" alt="RViz">
  <img src="https://img.shields.io/badge/TurtleBot3-Compatible-purple?style=for-the-badge&logo=robotframework&logoColor=white" alt="TurtleBot3">
</p>

A comprehensive autonomous patrol robot system built with ROS2, designed for TurtleBot3 simulation. This project implements an intelligent navigation system that allows a robot to autonomously patrol between predefined waypoints using Nav2 stack.

## 🤖 Features
- **Autonomous Waypoint Navigation**: Robot automatically patrols between predefined points
- **High-Speed Navigation**: Optimized navigation parameters for fast and efficient movement
- **Configurable Patrol Points**: Easy-to-modify waypoint configuration via YAML files
- **Nav2 Integration**: Full integration with ROS2 Navigation Stack
- **Simulation Ready**: Works seamlessly with TurtleBot3 Gazebo simulation
- **Real-time Visualization**: RViz integration for monitoring robot status
- **Safety Features**: Obstacle avoidance and collision detection

## 📋 Prerequisites
Before running this project, ensure you have the following installed:
- **ROS2 Humble** (or later)
- **TurtleBot3 packages**
- **Nav2 navigation stack**
- **Gazebo simulation environment**
- **RViz visualization tool**

### Required ROS2 Packages for project
```bash
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3-simulations
```

## 🚀 Quick Start
### 1. Clone and Build
```bash
# Create workspace
mkdir -p ~/ros2_patrol_ws/src
cd ~/ros2_patrol_ws/src
# Clone repository
git clone https://github.com/AungKaung1928/ros2-patrol-project.git
cd ~/ros2_patrol_ws
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
# Build the workspace
colcon build --packages-select patrol_navigation_project
source install/setup.bash
```

### 2. Set Environment Variables
```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_patrol_ws/src/ros2-patrol-project
```

### 3. Launch the System
**Terminal 1 - Gazebo Simulation:**
```bash
cd ~/ros2_patrol_ws
source install/setup.bash
ros2 launch patrol_navigation_project patrol_gazebo.launch.py
```

**Terminal 2 - Navigation and Patrol:**
```bash
cd ~/ros2_patrol_ws
source install/setup.bash
ros2 launch patrol_navigation_project patrol_navigation.launch.py
```

## 📁 Project Structure
```
src/patrol_navigation_project/
├── package.xml                             # Package dependencies and metadata
├── setup.py                               # Python package setup
├── setup.cfg                              # Setup configuration
│
├── config/                                # Configuration files
│   ├── nav2_params.yaml                   # Navigation parameters (high-speed)
│   └── patrol_points.yaml                 # Waypoint definitions
│
├── launch/                                # Launch files
│   ├── patrol_gazebo.launch.py           # Gazebo simulation launcher
│   └── patrol_navigation.launch.py       # Navigation stack launcher
│
├── patrol_navigation_project/             # Python package
│   ├── **init**.py                        # Package initialization
│   ├── patrol_controller.py               # Main patrol logic controller
│   └── waypoint_manager.py                # Waypoint management system
│
├── resource/                              # Resource files
│   └── patrol_navigation_project          # Package marker
│
├── rviz/                                  # RViz configurations
│   └── patrol_config.rviz                 # RViz visualization setup
│
└── test/                                  # Testing directory
    └── (test files)                       # Unit and integration tests
```

## ⚙️ Configuration
### Patrol Points Configuration
Edit `config/patrol_points.yaml` to customize patrol waypoints:
```yaml
patrol_points:
  - name: "checkpoint_1"
    x: 2.0
    y: 0.0
    z: 0.0
  - name: "checkpoint_2"
    x: 2.0
    y: 2.0
    z: 0.0
  - name: "checkpoint_3"
    x: 0.0
    y: 2.0
    z: 0.0
  - name: "checkpoint_4"
    x: 0.0
    y: 0.0
    z: 0.0
```

### Navigation Parameters
The project includes optimized navigation parameters in `config/nav2_params.yaml`:
- **High-Speed Movement**: `max_vel_x: 4.0`, `max_speed_xy: 4.0`
- **Aggressive Acceleration**: `acc_lim_x: 12.0`, `acc_lim_theta: 15.0`
- **Optimized Safety**: Balanced obstacle avoidance and speed
- **Smart Path Planning**: Efficient trajectory generation

### Key Configuration Sections
**Behavior Server Configuration:**
```yaml
behavior_server:
  ros__parameters:
    max_rotational_vel: 8.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 15.0
```

**BT Navigator Configuration:**
```yaml
bt_navigator:
  ros__parameters:
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    bt_loop_duration: 10
    default_server_timeout: 20
```

**Waypoint Follower Configuration:**
```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
```

## 🎮 Usage
### Basic Operation
1. **Start Simulation**: Launch Gazebo with TurtleBot3
   ```bash
   ros2 launch patrol_navigation_project patrol_gazebo.launch.py
   ```
2. **Initialize Navigation**: Start Nav2 stack with custom parameters
   ```bash
   ros2 launch patrol_navigation_project patrol_navigation.launch.py
   ```
3. **Monitor Progress**: Use RViz to visualize robot movement and navigation

### Advanced Controls
```bash
# Check patrol status
ros2 topic echo /patrol_status
# Stop patrol
ros2 service call /stop_patrol std_srvs/srv/Empty
# Resume patrol
ros2 service call /start_patrol std_srvs/srv/Empty
# Check current waypoint
ros2 topic echo /current_waypoint
# Monitor navigation parameters
ros2 param list /controller_server
```

## 🔧 Customization
### Adding New Waypoints
1. Edit `config/patrol_points.yaml`
2. Add new waypoint with name and coordinates:
   ```yaml
   - name: "new_checkpoint"
     x: 3.0
     y: 1.5
     z: 0.0
   ```
3. Rebuild: `colcon build --packages-select patrol_navigation_project`
4. Restart the navigation system

### Modifying Navigation Behavior
1. Edit `config/nav2_params.yaml`
2. Adjust speed limits, acceleration, or safety parameters
3. Key parameters to modify:
   - `max_rotational_vel`: Maximum rotation speed
   - `rotational_acc_lim`: Rotation acceleration
   - `bt_loop_duration`: Behavior tree update rate

### Custom RViz Configuration
1. Edit `rviz/patrol_config.rviz`
2. Add custom displays and panels
3. Save configuration for consistent visualization

## 🚨 Troubleshooting
### Common Issues
**Robot moves too slowly:**
- Check `config/nav2_params.yaml` speed settings
- Verify `max_rotational_vel` and acceleration parameters
- Ensure navigation stack is using updated parameters

**Navigation fails:**
- Verify map is loaded correctly
- Check patrol points are within map boundaries
- Ensure Nav2 stack is properly initialized
- Check TurtleBot3 model is set: `export TURTLEBOT3_MODEL=burger`

**Build errors:**
- Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Source ROS2 setup: `source /opt/ros/humble/setup.bash`
- Clean build: `rm -rf build install log && colcon build`

**Launch file not found:**
- Verify file paths match actual structure
- Check launch file permissions: `chmod +x launch/*.py`
- Ensure workspace is sourced: `source install/setup.bash`

### Debugging Commands
```bash
# Check node status
ros2 node list
# Monitor navigation topics
ros2 topic list | grep nav
# View transformation tree
ros2 run tf2_tools view_frames
# Check parameter values
ros2 param list /controller_server
ros2 param get /controller_server max_rotational_vel
# Verify package installation
ros2 pkg list | grep patrol_navigation_project
```
