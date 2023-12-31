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
