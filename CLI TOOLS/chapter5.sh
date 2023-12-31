#CHAPTER 5 UNDERSTANDING SERVICES

#Setting-up turtlebot, both must be run in seperate terminals.
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

ros2 service list
#Returns a list of available ROS2 service commands, must be run after the set-up has been done.

ros2 service type /clear
#This function returns

ros2 service list -t
#This command returns all active services.

ros2 service find std_srvs/srv/Empty
#This command returns empty type services.

ros2 interface show std_srvs/srv/Empty.srv
#this is used to call the empty service, as the empty service has no send/receive data it has no output.

ros2 interface show turtlesim/srv/Spawn
#This command outputs request and response arguments of 'Spawn'.

ros2 service call /clear std_srvs/srv/Empty
#Used to manually call the empty service, as it has no arguments, <arguments> after the code is not necessary.

ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
#Used to manually call the spawn function, unlike the function above, "{x: 2, y: 2, theta: 0.2, name: ''}" is used as an argument.
