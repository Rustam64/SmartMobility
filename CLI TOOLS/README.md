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

            turtlesim

<img width="621" alt="Chapter2 2" src="https://github.com/Rustam64/SmartMobility/assets/83468895/fe703f6c-21e3-40ee-9f97-2773121d6b67">

            turtle teleop key

## Chapter 2: Using turtlesim, ros2, and rqt

<img width="554" alt="Chapter2 4" src="https://github.com/Rustam64/SmartMobility/assets/83468895/af1c9637-46f4-4f45-9632-067b74cf4bb2">


            chapter2.4

Chapter 2 is quiet brief and goes over basic functions.  

The first 2 codes mentioned run the basic ROS-2 windows, refer to images chapter2.1 and chapter2.2

The 3rd code runs rqt, result can be seen on image chapter2.3  

Changine the pen color and size can be seen in chapter 2.4, the function can be found inside rqt under service caller.  

The last code runs a new teleop window to control the 2nd turtle. The result can be seen in chapter 2.5  


## Chapter 3: Understanding nodes

This chapter goes over nodes, how they are connected and how to get information on a given node.

The first code is a set-up to run turtlesim.

The 2nd code will return a list od nodes as can be seen in image chapter 3.1

The 3rd code will rename a node, in this case turtlesim to my_turtle.

The 4th will return detailed information on a node as can be seen in image chapter 3.2


## Chapter 4: Understanding topics

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

This chapter focuses on services which can either be ran thrugh rqt or the terminal.
