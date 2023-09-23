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
