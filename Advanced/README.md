- [Chapter 1: Chapter 1: Enabling Topic Statistics (C++)](#chapter-1-enabling-topic-statistics-c++)
- [Chapter 2: Using Fast DDS Discovery Server as a Discovery Protocol](#chapter-2-using-fast-dds-discovery-server-as-a-discovery-protocol)
- [Chapter 3: Implementing a Custom Memory Allocator](#chapter-3-implementing-a-custom-memory-allocator)
- [Chapter 4: Unlocking the Potential of Fast DDS Middleware](#chapter-4-unlocking-the-potential-of-fast-dds-middleware)
- [Chapter 5: Recording a Bag from a Node (C++)](#chapter-5-recording-a-bag-from-a-node-c)
- [Chapter 6: Recording a Bag from a Node (Python)](#chapter-6-recording-a-bag-from-a-node-python)
- [Chapter 7: Reading from a Bag File (C++)](#chapter-7-reading-from-a-bag-file-c)


You can click on any chapter to jump directly to the corresponding section in the README.

# Chapter 1: Enabling Topic Statistics (C++)

## Download the Listener File
```bash
wget -O member_function_with_topic_statistics.cpp [listener_file]
# Download the listener file from GitHub.
```

## Run the Subscriber Node
```bash
ros2 run cpp_pubsub listener_with_topic_statistics
# Run the subscriber node.
```

## Run the Talker Node
```bash
ros2 run cpp_pubsub talker
# Run the talker node.
```

## View Active Topics
```bash
ros2 topic list
# Show all active topics.
```

## View Statistics on Published Data
```bash
ros2 topic echo /statistics
# View statistics on published data.
```

---

# Chapter 2: Using Fast DDS Discovery Server as a Discovery Protocol

## Launch a Server
```bash
fastdds discovery --server-id 0
# Launching a server.
```

## Set Up ROS Discovery Server
```bash
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
# Setting up the ROS discovery server in a new terminal.
```

## Launch Listener Node
```bash
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server
# Launching the listener node.
```

## Launch Talker Node
```bash
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server
# Launching the talker node.
```

## Additional Configuration
You can explore various configurations for ROS discovery servers and nodes.

---

# Chapter 3: Implementing a Custom Memory Allocator

## Code for Custom Memory Allocator
- The code for the custom memory allocator can be found in [this link](https://github.com/ros2/demos/blob/humble/demo_nodes_cpp/src/topics/allocator_tutorial.cpp).

## Run Example Executable
```bash
ros2 run demo_nodes_cpp allocator_tutorial
# Run the example exec.
```

## Run Example Executable with Intra-Process Pipeline
```bash
ros2 run demo_nodes_cpp allocator_tutorial intra
# Run the example exec with the intra-process pipeline.
```

---

# Chapter 4: Unlocking the Potential of Fast DDS Middleware

## Package and Node Setup
1. Create a ROS 2 package and configure the necessary source files.
2. Set up the environment variables:
   ```bash
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export RMW_FASTRTPS_USE_QOS_FROM_XML=1
   export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml
   ```

## Run SyncAsyncWriter
```bash
ros2 run sync_async_node_example_cpp SyncAsyncWriter
# Source and run the node.
```

## Run SyncAsyncReader
```bash
ros2 run sync_async_node_example_cpp SyncAsyncReader
# Source and run the node.
```

## Create Ping Service and Client
- Add source files named `ping_service.cpp` and `ping_client.cpp`.
- Configure CMakeLists.txt to include ping_service and ping_client.

## Run Ping Service
```bash
ros2 run sync_async_node_example_cpp ping_service
# Run the service node.
```

## Run Ping Client
```bash
ros2 run sync_async_node_example_cpp ping_client
# Run the client node.
```

---

# Chapter 5: Recording a Bag from a Node (C++)

## Install ROS Bag
```bash
sudo apt install ros-humble-rosbag2
# Install rosbag.
```

## Create a ROS 2 Package
```bash
ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies example_interfaces rclcpp rosbag2_cpp std_msgs
# Create the package.
```

## Build and Run Bag Recorder Node
1. Build the package:
   ```bash
   colcon build --packages-select bag_recorder_nodes
   ```
2. Source the setup:
   ```bash
   source install/setup.bash
   ```
3. Run the node:
   ```bash
   ros2 run bag_recorder_nodes simple_bag_recorder
   ```
   Replace `/path/to/setup` with the appropriate setup information.

## Play the Recording
```bash
ros2 bag play my_bag
# Play the recording.
```

## Additional Data Generation and Playback
- For more data generation and playback options, refer to the provided source files and instructions.

---

# Chapter 6: Recording a Bag from a Node (Python)

## Create a Python Package
```bash
ros2 pkg create --build-type ament_python bag_recorder_nodes_py --dependencies rclpy rosbag2_py example_interfaces std_msgs
# Create the Python package.
```

## Build and Run Bag Recorder Node (Python)
1. Build the package:
   ```bash
   colcon build --packages-select bag_recorder_nodes_py
   ```
2. Source the setup:
   ```bash
   source install/setup.bash
   ```
3. Run the recorder node:
   ```bash
   ros2 run bag_recorder_nodes_py simple_bag_recorder
   ```
## Play the Recording
```bash
ros2 bag play my_bag
# Play the recording.
```

## Additional Data Generation and Playback (Python)
- For more data generation and playback options, refer to the provided source files and instructions.

---

# Chapter 7: Reading from a Bag File (C++)

## Create a ROS 2 Package
```bash
ros2 pkg create --build-type ament_cmake --license Apache-2
