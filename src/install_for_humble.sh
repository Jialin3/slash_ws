#!/bin/bash
sudo apt update -y
sudo apt install -y ros-humble-joint-state-publisher-gui
sudo apt install -y ros-humble-sdformat-urdf
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-ros-gz-sim
sudo apt install -y ros-humble-ros-gz-bridge
sudo apt install -y ros-humble-rqt-robot-steering 

#安装gazebo harmonic
sudo apt install -y curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
# sudo apt-get -y install gz-harmonic
sudo apt install -y ros-humble-ros-gzharmonic

#安装navigation2
sudo apt install -y ros-humble-navigation2 
sudo apt install -y ros-humble-slam-toolbox

