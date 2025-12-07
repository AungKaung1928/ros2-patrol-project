#!/bin/bash
# Quick Start Installation Script for Patrol Navigation Project (Python)
# Author: Allkg
# ROS2 Humble + Python 3.10+

set -e

echo "=========================================="
echo "ROS2 Patrol Navigation - Quick Install"
echo "=========================================="

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check ROS2 installation
echo -e "${YELLOW}Checking ROS2 installation...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
  echo -e "${GREEN}✓ ROS2 Humble found${NC}"
  source /opt/ros/humble/setup.bash
else
  echo -e "${RED}✗ ROS2 Humble not found. Please install ROS2 Humble first.${NC}"
  exit 1
fi

# Check Python version
echo -e "${YELLOW}Checking Python version...${NC}"
PYTHON_VERSION=$(python3 --version | awk '{print $2}')
PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)

if [ "$PYTHON_MAJOR" -ge 3 ] && [ "$PYTHON_MINOR" -ge 10 ]; then
  echo -e "${GREEN}✓ Python ${PYTHON_VERSION} found${NC}"
else
  echo -e "${RED}✗ Python 3.10+ required. Found: ${PYTHON_VERSION}${NC}"
  exit 1
fi

# Install dependencies
echo -e "${YELLOW}Installing ROS2 dependencies...${NC}"
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3* \
  ros-humble-visualization-msgs \
  python3-yaml \
  python3-colcon-common-extensions \
  python3-pytest

echo -e "${GREEN}✓ Dependencies installed${NC}"

# Create workspace
WORKSPACE_DIR="$HOME/ros2_patrol_ws"
echo -e "${YELLOW}Creating workspace at ${WORKSPACE_DIR}...${NC}"

if [ -d "$WORKSPACE_DIR" ]; then
  echo -e "${YELLOW}Workspace already exists. Using existing workspace.${NC}"
else
  mkdir -p $WORKSPACE_DIR/src
  echo -e "${GREEN}✓ Workspace created${NC}"
fi

# Copy package to workspace
echo -e "${YELLOW}Copying patrol_navigation_project to workspace...${NC}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ -d "$WORKSPACE_DIR/src/patrol_navigation_project" ]; then
  echo -e "${YELLOW}Removing old package...${NC}"
  rm -rf $WORKSPACE_DIR/src/patrol_navigation_project
fi

cp -r $SCRIPT_DIR $WORKSPACE_DIR/src/patrol_navigation_project
echo -e "${GREEN}✓ Package copied${NC}"

# Build workspace
echo -e "${YELLOW}Building workspace...${NC}"
cd $WORKSPACE_DIR
colcon build --packages-select patrol_navigation_project --symlink-install

if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ Build successful${NC}"
else
  echo -e "${RED}✗ Build failed${NC}"
  exit 1
fi

# Setup environment
echo -e "${YELLOW}Setting up environment...${NC}"

# Add to bashrc if not already present
if ! grep -q "export TURTLEBOT3_MODEL=burger" ~/.bashrc; then
  echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
  echo -e "${GREEN}✓ Added TURTLEBOT3_MODEL to .bashrc${NC}"
fi

if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
  echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
  echo -e "${GREEN}✓ Added workspace to .bashrc${NC}"
fi

# Source workspace
source $WORKSPACE_DIR/install/setup.bash

echo ""
echo "=========================================="
echo -e "${GREEN}Installation Complete!${NC}"
echo "=========================================="
echo ""
echo "To run the patrol navigation system:"
echo ""
echo "Terminal 1 - Launch Gazebo:"
echo "  export TURTLEBOT3_MODEL=burger"
echo "  ros2 launch patrol_navigation_project patrol_gazebo.launch.py"
echo ""
echo "Terminal 2 - Launch Navigation (wait 30s after Terminal 1):"
echo "  cd ~/ros2_patrol_ws"
echo "  source install/setup.bash"
echo "  ros2 launch patrol_navigation_project patrol_navigation.launch.py"
echo ""
echo "For more info, see README.md"
echo ""