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
