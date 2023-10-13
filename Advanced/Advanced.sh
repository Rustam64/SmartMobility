Chapter 1. Enabling topic statistics (C++)

wget -O member_function_with_topic_statistics.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function_with_topic_statistics.cpp
#Downloading the listener file from github.

#Edit the CMakeLists.txt to add the new executable.

ros2 run cpp_pubsub listener_with_topic_statistics
# Run the subscriber node

ros2 run cpp_pubsub talker
#Run the talker node

ros2 topic list
# Show all active topics

ros2 topic echo /statistics
# View statistics on published data.


Chapter 2. Using Fast DDS Discovery Server as discovery protocol

fastdds discovery --server-id 0
#Launching a server

export ROS_DISCOVERY_SERVER=127.0.0.1:11811
#Setting up the Ros discovery server in a new terminal.

ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server
#Launchign the listener node

export ROS_DISCOVERY_SERVER=127.0.0.1:11811
#Setting up the Ros discovery server in a new terminal.

ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server
#Launching the talker node

ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener
#Launching a simple listener in a new terminal.

ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker
#Launching a simple talker in a new terminal.

#Run all the codes below in seperate terminals:

fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811
#Creating a server

fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888
#Creating a secondary server

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
#Setting up the Ros discovery server.
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
#Launching the talker node

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
#Setting up the Ros discovery server.
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
#Launching the listener node

#Run all the codes below in seperate terminals:

fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811 --backup
#Creating a backup server

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
#Setting up the Ros discovery server.
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
#Launchign the talker node

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
#Setting up the Ros discovery server.
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
#Launching the listener node.

fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811
#Launch a server

fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888
#Launch a secondary server

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_1
#Talker node connected to 11811 and 11888

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_1
#Listener node connected to 11811 and 11888

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_2
#Talker node connected to 11811

export ROS_DISCOVERY_SERVER=";127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_2
#Listener node connected to 11888
#As listener 2 is on 11888 and talker 2 is on 11811 they do not communicate.

fastdds discovery -i 0 -l 127.0.0.1 -p 11811
#Launch a discovery server

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
#Launch a listener node

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
#Launch a talker node

export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 daemon stop
ros2 daemon start
ros2 topic list
ros2 node info /talker
ros2 topic info /chatter
ros2 topic echo /chatter
#instantiate a ROS 2 Daemon using the Super Client configuration

export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
rqt_graph
#Node graph using rqt


Chapter 3. Implementing a custom memory allocator
#The code for the custom memory allocator can be found in the link below:
https://github.com/ros2/demos/blob/humble/demo_nodes_cpp/src/topics/allocator_tutorial.cpp

ros2 run demo_nodes_cpp allocator_tutorial
# Run example exec

ros2 run demo_nodes_cpp allocator_tutorial intra
# Run example exec with intra-process pipeline


Chapter 4. Unlocking the potential of Fast DDS middleware

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs -- sync_async_node_example_cpp
#Create a directory and a package.

#create a file named src/sync_async_writer.cpp

#In CMakeLists.txt add SyncAsyncWriter executable

#Create SyncAsync.xml.

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml
# Export environment variables

source install/setup.bash
ros2 run sync_async_node_example_cpp SyncAsyncWriter
#Source and run the noce

#Create src/sync_async_reader.cpp

#In CMakeLists.txt add SyncAsyncReader executable

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml
# Export environment variables

source install/setup.bash
ros2 run sync_async_node_example_cpp SyncAsyncReader
#Source and run the noce

#Add source file named src/ping_service.cpp

#Create the client in src/ping_client.cpp

#In CMakeLists.txt add ping_service and ping_client

#Create ping.xml

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/ping.xml
# Export environment variables

ros2 run sync_async_node_example_cpp ping_service
#Run the service node

ros2 run sync_async_node_example_cpp ping_client
#Run the client node


Chapter 5. Recording a bag from a node

sudo apt install ros-humble-rosbag2
#Install rosbag

ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies example_interfaces rclcpp rosbag2_cpp std_msgs
#Create the package

#Update the package.xml file

#In ros2_ws/src/bag_recorder_nodes/src create a file named simple_bag_recorder.cpp

#In CMakeLists.txt change CMAKE_CXX_STANDARD to 17

#In CMakeLists.txt add executable simple_bag_recorder

colcon build --packages-select bag_recorder_nodes
#Build the package

source install/setup.bash
#Source the setup

ros2 run bag_recorder_nodes simple_bag_recorder
#Run the node that records

ros2 run demo_nodes_cpp talker
#Run the talker node

ros2 run demo_nodes_cpp listener
#Run the listener

ros2 bag play my_bag
#Play the recording

#In ros2_ws/src/bag_recorder_nodes/src create data_generator_node.cpp

#In CMakeLists.txt add data_generator_node executable

colcon build --packages-select bag_recorder_nodes
#Build the package

source install/setup.bash
#Source set-up

ros2 run bag_recorder_nodes data_generator_node
#Run the generator

ros2 bag play timed_synthetic_bag
#Play the data

ros2 topic echo /synthetic
#Print data

#In ros2_ws/src/bag_recorder_nodes/src create data_generator_executable.cpp

#Add data_generator_executable to CMakeLists.txt

colcon build --packages-select bag_recorder_nodes
#Build the package

source install/setup.bash
#Source set-up

ros2 run bag_recorder_nodes data_generator_executable
#Run the data generator

ros2 bag play big_synthetic_bag
#Play the data

ros2 topic echo /synthetic
#Print data

Chapter 6. Recording a bag from a node (Python)

ros2 pkg create --build-type ament_python bag_recorder_nodes_py --dependencies rclpy rosbag2_py example_interfaces std_msgs
#create the python package

#Update package.xml and setup.py

#Inside ros2_ws/src/bag_recorder_nodes_py/bag_recorder_nodes_py directory, create a new file called simple_bag_recorder.py

#In setup.py add an entry point for your node

colcon build --packages-select bag_recorder_nodes_py
#build the package

source install/setup.bash
#Source the set-up

ros2 run bag_recorder_nodes_py simple_bag_recorder
#Run the recorder

ros2 run demo_nodes_cpp talker
#Run the talker node

ros2 run demo_nodes_cpp listener
#Run the listener

ros2 bag play my_bag
#Play the recording

#In ros2_ws/src/bag_recorder_nodes_py/bag_recorder_nodes_py directory, create a new file called data_generator_node.py

#In setup.py add an entry point for your node

colcon build --packages-select bag_recorder_nodes_py
#build the package

source install/setup.bash
#Source the set-up

ros2 run bag_recorder_nodes_py data_generator_node
#Run the generator node

ros2 bag play timed_synthetic_bag
#After terminating the generator, run the player

ros2 topic echo /synthetic
#Echo the synthetic

#In the ros2_ws/src/bag_recorder_nodes_py/bag_recorder_nodes_py directory, create a new file called data_generator_executable.py

#In setup.py add an entry point for your node

colcon build --packages-select bag_recorder_nodes_py
#build the package

source install/setup.bash
#Source the set-up

ros2 run bag_recorder_nodes_py data_generator_executable
#Run the new generator

ros2 bag play big_synthetic_bag
#After the generator automatically shuts-down play the recording

ros2 topic echo /synthetic
#Echo the synthetic data

Chapter 7. Reading from a bag file (C++)

ros2 pkg create --build-type ament_cmake --license Apache-2.0 bag_reading_cpp --dependencies rclcpp rosbag2_cpp turtlesim
#Create the package

#Update package.xml

#In src create simple_bag_reader.cpp

#In CMakeLists.txt add simple_bag_reader executable

colcon build --packages-select bag_reading_cpp
#Build the package

source install/setup.bash
#Source the install

ros2 run bag_reading_cpp simple_bag_reader /path/to/setup
#Run the node
