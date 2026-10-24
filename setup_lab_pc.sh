#!/bin/bash
set -e

echo "=== Installing System & ROS 2 Dependencies ==="
sudo apt update
sudo apt install -y \
  python3-numpy \
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

echo "=== Configuring DDS Environment Variables in ~/.bashrc ==="
if ! grep -q "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
fi

if ! grep -q "CYCLONEDDS_URI" ~/.bashrc; then
    echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc
fi

echo "=== Populating Local Gazebo Models (~/.gazebo/models) ==="
mkdir -p ~/.gazebo/models
if [ ! -d "$HOME/.gazebo/models/sun" ] || [ ! -d "$HOME/.gazebo/models/ground_plane" ]; then
    git clone --depth 1 https://github.org/osrf/gazebo_models.git ~/.gazebo/models_tmp
    cp -r ~/.gazebo/models_tmp/sun ~/.gazebo/models/ 2>/dev/null || true
    cp -r ~/.gazebo/models_tmp/ground_plane ~/.gazebo/models/ 2>/dev/null || true
    rm -rf ~/.gazebo/models_tmp
fi

echo "=== Setup Complete! Run 'source ~/.bashrc' before building ==="