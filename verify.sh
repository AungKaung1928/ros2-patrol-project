#!/bin/bash
# Package Verification Script
# Checks if all required files are present and valid

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=========================================="
echo "Patrol Navigation Project - Package Verification"
echo "=========================================="

# Function to check file exists
check_file() {
  if [ -f "$1" ]; then
    echo -e "${GREEN}✓${NC} $1"
    return 0
  else
    echo -e "${RED}✗${NC} $1 MISSING"
    return 1
  fi
}

# Function to check directory exists
check_dir() {
  if [ -d "$1" ]; then
    echo -e "${GREEN}✓${NC} $1/"
    return 0
  else
    echo -e "${RED}✗${NC} $1/ MISSING"
    return 1
  fi
}

ERRORS=0

echo ""
echo "Checking core files..."
check_file "setup.py" || ((ERRORS++))
check_file "setup.cfg" || ((ERRORS++))
check_file "package.xml" || ((ERRORS++))
check_file "README.md" || ((ERRORS++))

echo ""
echo "Checking Python package..."
check_dir "patrol_navigation_project" || ((ERRORS++))
check_file "patrol_navigation_project/__init__.py" || ((ERRORS++))
check_file "patrol_navigation_project/patrol_controller.py" || ((ERRORS++))
check_file "patrol_navigation_project/waypoint_manager.py" || ((ERRORS++))
check_file "patrol_navigation_project/patrol_visualizer.py" || ((ERRORS++))

echo ""
echo "Checking configuration..."
check_dir "config" || ((ERRORS++))
check_file "config/nav2_params.yaml" || ((ERRORS++))
check_file "config/patrol_points.yaml" || ((ERRORS++))

echo ""
echo "Checking launch files..."
check_dir "launch" || ((ERRORS++))
check_file "launch/patrol_gazebo.launch.py" || ((ERRORS++))
check_file "launch/patrol_navigation.launch.py" || ((ERRORS++))

echo ""
echo "Checking tests..."
check_dir "test" || ((ERRORS++))
check_file "test/test_copyright.py" || ((ERRORS++))
check_file "test/test_flake8.py" || ((ERRORS++))
check_file "test/test_pep257.py" || ((ERRORS++))

echo ""
echo "Checking resource files..."
check_dir "resource" || ((ERRORS++))
check_file "resource/patrol_navigation_project" || ((ERRORS++))

echo ""
echo "Checking documentation..."
check_file "RUN_COMMANDS.md" || ((ERRORS++))
check_file "CPP_VS_PYTHON.md" || ((ERRORS++))
check_file "install.sh" || ((ERRORS++))

echo ""
echo "Checking Python syntax..."
for py_file in patrol_navigation_project/*.py launch/*.py test/*.py; do
  if [ -f "$py_file" ]; then
    if python3 -m py_compile "$py_file" 2>/dev/null; then
      echo -e "${GREEN}✓${NC} $py_file syntax OK"
    else
      echo -e "${RED}✗${NC} $py_file syntax ERROR"
      ((ERRORS++))
    fi
  fi
done

echo ""
echo "Checking YAML syntax..."
for yaml_file in config/*.yaml; do
  if [ -f "$yaml_file" ]; then
    if python3 -c "import yaml; yaml.safe_load(open('$yaml_file'))" 2>/dev/null; then
      echo -e "${GREEN}✓${NC} $yaml_file syntax OK"
    else
      echo -e "${RED}✗${NC} $yaml_file syntax ERROR"
      ((ERRORS++))
    fi
  fi
done

echo ""
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
  echo -e "${GREEN}✓ All checks passed!${NC}"
  echo "Package is ready for installation."
  exit 0
else
  echo -e "${RED}✗ Found $ERRORS error(s)${NC}"
  echo "Please fix the issues before installation."
  exit 1
fi