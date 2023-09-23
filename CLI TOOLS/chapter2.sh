#CHAPTER 2: Using turtlesim, ros2, and rqt
ros2 run turtlesim turtlesim_node
#To create a UI for the TurtleSim and view the turtle position.

ros2 run turtlesim turtle_teleop_key
#Run in a new terminal to move the turtle using the keyboard.

rqt
#To run rqt, a command panel used to spawn and modify the turtles.

ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
#To open a new teleop key to control the 2nd turtle.
