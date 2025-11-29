#!/bin/bash
set -e

PASSWORD="123456"

echo "--- Setting Locale ---"
echo $PASSWORD | sudo -S apt update && sudo apt install locales -y
echo $PASSWORD | sudo -S locale-gen en_US en_US.UTF-8
echo $PASSWORD | sudo -S update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "--- Adding ROS2 Sources ---"
echo $PASSWORD | sudo -S apt install software-properties-common -y
echo $PASSWORD | sudo -S add-apt-repository universe -y
echo $PASSWORD | sudo -S apt update && sudo apt install curl -y
echo $PASSWORD | sudo -S curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "--- Installing ROS2 Humble ---"
echo $PASSWORD | sudo -S apt update
echo $PASSWORD | sudo -S apt install ros-humble-desktop -y
echo $PASSWORD | sudo -S apt install ros-dev-tools -y

echo "--- Installing Python Dependencies ---"
echo $PASSWORD | sudo -S apt install python3-pip python3-venv -y

echo "--- Setup Complete ---"
