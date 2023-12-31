#CHAPTER 9 LAUNCHING NODES
ros2 launch turtlesim multisim.launch.py
#The command above will launch 2 turtlesim windows.

#To control turtlesim1 the command prior and to control turtlesim2 the command use the following command.

ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
