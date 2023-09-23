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
