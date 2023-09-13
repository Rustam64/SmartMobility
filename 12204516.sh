The Shebang Line
#!/bin/bash

$chmod +x myscript.sh

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

#Ensure that the Ubuntu Universe repository is enabled.
sudo apt install software-properties-common
sudo add-apt-repository universe

#Add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#Add the repository to sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#Update your apt repository caches after setting up the repositories.
sudo apt update

#Install 
sudo apt install ros-foxy-desktop python3-argcomplete

#ROS-Base Install 
sudo apt install ros-foxy-ros-base python3-argcomplete

#Install Development tools
sudo apt install ros-dev-tools

source /opt/ros/foxy/setup.bash

source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker

source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener