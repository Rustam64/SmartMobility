# ROS2 TurtleSim Tutorial

This repository contains code and instructions for working with the TurtleSim simulation in ROS2, as well as other related concepts such as nodes, topics, services, parameters, actions, and recording/playback of data. All code is provided in seperate shell files, however the code must be run manually in seperate terminals. The comments will provide information on the code and the readme will link it to viable screenshots.

## Table of Contents

1. [Chapter 2: Using turtlesim, ros2, and rqt](#chapter-2-using-turtlesim-ros2-and-rqt)
2. [Chapter 3: Understanding nodes](#chapter-3-understanding-nodes)
3. [Chapter 4: Understanding topics](#chapter-4-understanding-topics)
4. [Chapter 5: Understanding services](#chapter-5-understanding-services)
5. [Chapter 6: Understanding parameters](#chapter-6-understanding-parameters)
6. [Chapter 7: Actions](#chapter-7-actions)
7. [Chapter 8: Viewing Logs with rqt_console](#chapter-8-viewing-logs-with-rqt_console)
8. [Chapter 9: Launching Nodes](#chapter-9-launching-nodes)
9. [Chapter 10: Recording and Playing Back Data](#chapter-10-recording-and-playing-back-data)


## Chapter 1: Environment set-up.
Brief code to set-up the ROS and other settings.
Turtlesim and turtle tele key can be seen in images below:

<img width="617" alt="Chapter2 1" src="https://github.com/Rustam64/SmartMobility/assets/83468895/96c5e2fa-65af-4495-a3ef-a970eea863a0">

            Turtlesim

<img width="621" alt="Chapter2 2" src="https://github.com/Rustam64/SmartMobility/assets/83468895/fe703f6c-21e3-40ee-9f97-2773121d6b67">

            Turtle teleop key

## Chapter 2: Using turtlesim, ros2, and rqt

<img width="554" alt="Chapter2 4" src="https://github.com/Rustam64/SmartMobility/assets/83468895/af1c9637-46f4-4f45-9632-067b74cf4bb2">


            ROS-2 graph after changing pen and spawning a 2nd turtle.

Chapter 2 is quiet brief and goes over basic functions.  

The first 2 codes mentioned run the basic ROS-2 windows, refer to images chapter2.1 and chapter2.2

The 3rd code runs rqt, result can be seen on image chapter2.3  

Changine the pen color and size can be seen in chapter 2.4, the function can be found inside rqt under service caller.  

The last code runs a new teleop window to control the 2nd turtle. The result can be seen in chapter 2.5  


## Chapter 3: Understanding nodes

<img width="300" alt="chapter3 1" src="https://github.com/Rustam64/SmartMobility/assets/83468895/7b2b936f-03c2-4bc9-bd73-e4386c516886">

            List of nodes.


This chapter goes over nodes, how they are connected and how to get information on a given node.

The first code is a set-up to run turtlesim.

The 2nd code will return a list od nodes as can be seen in image chapter 3.1

The 3rd code will rename a node, in this case turtlesim to my_turtle.

The 4th will return detailed information on a node as can be seen in image chapter 3.2


## Chapter 4: Understanding topics

<img width="350" alt="chapter4 5" src="https://github.com/Rustam64/SmartMobility/assets/83468895/cd25a2a5-6276-4b10-af01-73da7363686c">

            Ros Graph.


This chapter is about topics in ROS2, how to view and manipulate them.

The result of 'rqt_graph' can be seen on image chapter 4.1

The result of 'ros2 topic list' can be seen on image chapter 4.2

The result of 'ros2 topic list -t' can be seen on image chapter 4.3

Output message of 'ros2 topic echo /turtle1/cmd_vel' can be seen on image chapter 4.4

An example graph of all nodes can be seen on image chapter 4.5

The result of 'ros2 topic info /turtle1/cmd_vel' can be seen on image chapter 4.6

The result of 'ros2 interface show geometry_msgs/msg/Twist' can be seen on image chapter 4.7

To see how the turtle moved after publishing data to the nodes see images chapter 4.8 and chapter 4.9

An example of 'ros2 topic hz /turtle1/pose' can be seen on image chapter 4.10




## Chapter 5: Understanding services


<img width="370" alt="chapter5 2" src="https://github.com/Rustam64/SmartMobility/assets/83468895/0ba0bc4c-cffb-4cf2-a77a-b3ec58aeb935">

            Service List.

This chapter focuses on services which can either be ran thrugh rqt or the terminal.


To get a list of services as can be seen above 'ros2 service list' as well as adding '-t' at the end will provide extra details.

To call the empty service 'ros2 interface show std_srvs/srv/Empty.srv' is used , as the empty service has no send/receive data it has no output.

This command outputs request and response arguments of 'Spawn' as can be seen in image chapter 5.3 'ros2 interface show turtlesim/srv/Spawn' 

To manually call empty service as can be seen in image chapter 5.4 'ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"' 

## Chapter 6: Understanding Parameters

<img width="623" alt="chapter6 3" src="https://github.com/Rustam64/SmartMobility/assets/83468895/606b44b7-c9ce-458d-b34f-4260f149ddf4">

            Turtlesim after the background has been changed to purple.

This chapter covers the use of parameters in ROS2. It includes commands to list parameters, get and set parameter values, and save/load parameters. Additionally, it demonstrates how to run the TurtleSim node with preset parameters.

To list parameters use 'ros2 param list' as in image chapter 6.1

To get a parameter value use 'ros2 param get /turtlesim background_g' as seen in image chapter 6.2.

To edit a parameter use 'ros2 param set /turtlesim background_r 150' as in image chapter 6.3

To save parameters use 'ros2 param dump /turtlesim' and to load parameters use '# ros2 param load /turtlesim ./turtlesim.yaml'

Lastly, to run ROS with preset parameters use 'ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml'


## Chapter 7: Actions

<img width="621" alt="chapter7 4" src="https://github.com/Rustam64/SmartMobility/assets/83468895/96eb62de-fc00-4c79-b7be-7ac6966bb486">

            Rotating the turtle manually by sending data via command line.

To get a list of actions use 'ros2 action list' for reference see image chapter 7.2.

To get information about an action use ros2 action info /turtle1/<name of action>

To manually request an action use 'ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"' see images chapter 7.3 and 7.4

Lastly, providing feedback by 'ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback'


## Chapter 8: Viewing Logs with rqt_console

<img width="655" alt="chapter8 2" src="https://github.com/Rustam64/SmartMobility/assets/83468895/c6e93ccc-da73-47c3-ab30-78252dc4991f">

            rqt console with feedback from the turtlesim and turtle tele-key.

To run rqt_console: 'ros2 run rqt_console rqt_console' as can be seen in image chapter 8.1

To move the turtle via cmd 'ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'

The result of the following can be seen in the image chapter 8.2

## Chapter 9: Launching Nodes

<img width="503" alt="chapter9 1" src="https://github.com/Rustam64/SmartMobility/assets/83468895/72673d5f-78e4-41a4-8606-d2f85838dd88">

            Multisim launch.

To create 2 turtlesim windows you can use 'ros2 launch turtlesim multisim.launch.py'

To control TurtleSim1: 'ros2 topic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"'

To control TurtleSim2: 'ros2 topic pub /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"'

The output can be seen in the image above.

## Chapter 10: Recording and Playing Back Data

<img width="620" alt="chapter10 2" src="https://github.com/Rustam64/SmartMobility/assets/83468895/1cdad10b-52e3-4656-a493-ae59175b1f2a">

            Recording ROS2 data.

Follow instructions set in the shell file until line 16 to set-up the packages and environment.

Run topic list to get a list of topics to choose from and then use 'ros2 topic echo <topic>' to capture data. An example can be seen in image chapter 10.1

You should use 'ros2 bag record <topic>' to record data. An example can be seen in image chapter 10.2

The following command is an example on how to change name(-o) and record multiple topics. 'ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose'

You can get information on the recording using 'ros2 bag info <recording name>' An example can be seen in image chapter 10.3

You can play a recording using 'ros2 bag play <recording name>' An example can be seen in image chapter 10.4









