

#CHAPTER 2: Using turtlesim, ros2, and rqt

ros2 run turtlesim turtlesim_node
#To create a UI for the TurtleSim and view the turtle position.

ros2 run turtlesim turtle_teleop_key
#Run in a new terminal to move the turtle using the keyboard.

rqt
#To run rqt, a command panel used to spawn and modify the turtles.

ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
#To open a new teleop key to control the 2nd turtle.


#CHAPTER 3: Understanding nodes

ros2 run turtlesim turtlesim_node
#Here the 'turtlesim' is the package and 'turtlesim_node' is the executable.

ros2 node list
#Run this command in a new terminal to see active nodes.


#CHAPTER 4: UNDERSTANDING TOPICS

ros2 run turtlesim turtlesim_node
#Run this in a new terminal to run the turtle interface.

ros2 run turtlesim turtle_teleop_key
#Run this in a new terminal to control the turtle.

rqt_graph
#Run this in a new terminal to see nodes, topics and connections.

ros2 topic list
#Run this in a new terminal, this command shows all active nodes.

ros2 topic list -t
#Run this in a new terminal, this command shows all active nodes with their topic type.

ros2 topic echo /turtle1/cmd_vel
#Run this command on any unactive terminal to see message on  this node.

ros2 topic info /turtle1/cmd_vel
#Run this on unactive terminal to see publishers and subscribers on this node.

ros2 interface show geometry_msgs/msg/Twist
#Run this on an unactive terminal to see details about what data the node expects.

ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
#This command can be run to manually send data to the turtlesim to turn the turtle.

ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
#This command is similar to the former, however, will make the turtle move in a circle indefinetely.

ros2 topic hz /turtle1/pose
#This command is used to determine the rate at which turtlesim publishes data to the pose topic.


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


#CHAPTER 6 UNDERSTANDING PARAMETERS

#Before starting the 2 lines below must be run in seperate terminals and every following line in yet another terminal.
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

ros2 param list
#Results parameters of nodes.

ros2 param get /turtlesim background_g
#This returns the value of the background parameter(86, a color)

ros2 param set /turtlesim background_r 150
#This sets the background value to 150, changing the background color to purple.

#IF you wish to save the parameters you can use the function below.
ros2 param dump /turtlesim

#ros2 param load /turtlesim ./turtlesim.yaml
#This function is used to load previously saved parameters.

ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
#Function used to load turtlesim with preset parameters.


#CHAPTER 7 ACTIONS

#Before starting the 2 lines below must be run in seperate terminals and every following line in yet another terminal.
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

ros2 node info /turtlesim
#This function will return a list of turtlesim’s subscribers, publishers, services, action servers and action clients.

ros2 node info /teleop_turtle
#Provide data on the teleop turtle.

ros2 action list
#Actions for the ROS graph.

ros2 action list -t
#Actions and action types for the ROS graph.

ros2 action info /turtle1/rotate_absolute
# action info give information about the following action.

ros2 interface show turtlesim/action/RotateAbsolute
#Gives structure of request, result and feedback.

ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
#Manually sending a request for the turtle to turn.

ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
#Providing feedback for an action.



#CHAPTER 8 RQT_CONSOLVE TO VIEW LOGS
#As usual, before starting, the 2 lines below must be run in seperate terminals and every following line in yet another terminal.
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

ros2 run rqt_console rqt_console
#To run the rqt_console.

ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
#Sending a request to turtlesim to move the robot in a loop.

ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
#This command makes it so that the turtlesim only sends WARN level messages.



#CHAPTER 9 LAUNCHING NODES
ros2 launch turtlesim multisim.launch.py
#The command above will launch 2 turtlesim windows.

#To control turtlesim1 the command prior and to control turtlesim2 the command use the following command.

ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"



#CHAPTER 10 RECORDING AND PLAYING BACK DATA

#Before starting,download necessary packages using the command below.
sudo apt-get install ros-foxy-ros2bag \
                     ros-foxy-rosbag2-converter-default-plugins \
                     ros-foxy-rosbag2-storage-default-plugins
                     
#Use the 2 commands below in seperate terminals to set-up necessary windows.
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

mkdir bag_files
cd bag_files
#The 2 commands above create a new directory names bag_files and move the terminal to that directory.

ros2 topic list
#A command previously used. Gives a list of topic list.

ros2 topic echo /turtle1/cmd_vel
#Captures data send out by teleop.

ros2 bag record /turtle1/cmd_vel
#This command saves data from the turtle to the folder/directory at the terminal.

ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
#This command saves data from multiple topics.The -o option is to change the name of the file.

ros2 bag info subset
#Gives information on the recording

ros2 bag play subset
#Plays the mentioned recording.
