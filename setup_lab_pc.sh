#!/bin/bash
set -e

echo "=== Installing System, ROS 2 & Controller Dependencies ==="
sudo apt update
sudo apt install -y \
  git \
  python3-numpy \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-diff-drive-controller \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-teleop-twist-keyboard \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-irobot-create-description \
  ros-humble-irobot-create-control \
  ros-humble-irobot-create-msgs \
  ros-humble-irobot-create-nodes \
  ros-humble-irobot-create-toolbox

echo "=== Configuring CycloneDDS (~/cyclonedds.xml) ==="
cat << 'XML' > ~/cyclonedds.xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/2010/05/cyclonedds" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/2010/05/cyclonedds https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
XML

echo "=== Disabling Gazebo Online Model Database Lookup (Fixes Launch Timeouts) ==="
mkdir -p ~/.gazebo
echo "GAZEBO_MODEL_DATABASE_URI=" > ~/.gazebo/config

echo "=== Populating Local Gazebo Models (~/.gazebo/models) ==="
mkdir -p ~/.gazebo/models
if [ ! -d "$HOME/.gazebo/models/sun" ] || [ ! -d "$HOME/.gazebo/models/ground_plane" ]; then
    git clone --depth 1 https://github.org/osrf/gazebo_models.git ~/.gazebo/models_tmp
    cp -r ~/.gazebo/models_tmp/sun ~/.gazebo/models/ 2>/dev/null || true
    cp -r ~/.gazebo/models_tmp/ground_plane ~/.gazebo/models/ 2>/dev/null || true
    rm -rf ~/.gazebo/models_tmp
fi

echo "=== Configuring Environment Variables & Paths in ~/.bashrc ==="

# CycloneDDS environment setup
if ! grep -q "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
fi

if ! grep -q "CYCLONEDDS_URI" ~/.bashrc; then
    echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc
fi

# System Gazebo sourcing
if ! grep -q "gazebo/setup.bash" ~/.bashrc; then
    echo 'source /usr/share/gazebo/setup.bash 2>/dev/null || source /usr/share/gazebo-11/setup.bash' >> ~/.bashrc
fi

# Gazebo resource & plugin path exports (Preserves system paths while adding ROS 2 share)
if ! grep -q "GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib" ~/.bashrc; then
    echo 'export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
fi

if ! grep -q "GAZEBO_MODEL_PATH=/opt/ros/humble/share" ~/.bashrc; then
    echo 'export GAZEBO_MODEL_PATH=/opt/ros/humble/share:$GAZEBO_MODEL_PATH' >> ~/.bashrc
fi

if ! grep -q "GAZEBO_RESOURCE_PATH=/opt/ros/humble/share" ~/.bashrc; then
    echo 'export GAZEBO_RESOURCE_PATH=/opt/ros/humble/share:$GAZEBO_RESOURCE_PATH' >> ~/.bashrc
fi

echo "=== Setup Complete! Run 'source ~/.bashrc' before building your workspace ==="