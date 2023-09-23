#CHAPTER 1: Configuring environment
source /opt/ros/foxy/setup.bash
#This sources ROS, must be used at the beginning of a shell file for it to work.

echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
#This can be used to add sourcing to shell start-up.

printenv | grep -i ROS
#To test ROS 2 installation.

ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=foxy
#To check taht ROS_DISCTRO AND ROS_VERSION are working as intended.

export ROS_DOMAIN_ID=1
#Setting the environment variable.

echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
#This maintains settings betweens sessions.

export ROS_LOCALHOST_ONLY=1
#Allows ROS to communicate only with the local host.

echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
#Yet again, to maintain this setting between shell sessions.
