## Chapter 1: Using Colcon to Build Packages

<img width="368" alt="2" src="https://github.com/Rustam64/SmartMobility/assets/83468895/f9d7816e-96a1-4a9f-8278-58a1292f0615">

    Building a package using Colcon.

The first chapter focuses on installing packages and setting-up the environment.

You need to first install colcon using sudo apt and then create a directory for it.

Next, You need to clone a git repository which downloads some files which will be used later. Refer to image chapter 1.1

To build a test package use 'colcon build --symlink-install' Refer to image chapter 1.2

Once all is done, you need to source using 'source install/setup.bash' 

Next, run the subscriber node using 'ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function'

Also run the publisher node using 'ros2 run examples_rclcpp_minimal_publisher publisher_member_function' Refer to image chapter 1.4

Lastly set-up a colcon CD and Tab completion.


## Chapter 2 Creating a Workspace

<img width="816" alt="2" src="https://github.com/Rustam64/SmartMobility/assets/83468895/69b0a23e-e7d8-43e7-97e4-b7c56f49a591">

    Packages files after building a package.

First of all, the ROS2 environment has to be sourced and a directory created.

Afterwards, download the repository as shown and install dependencies, this can be seen in image 1 of chapter 2.

Lastly, build the package. The resulting folder should look like in image 2 of chapter 2.

## Chapter 3 Creating a package

<img width="349" alt="thumbnail" src="https://github.com/Rustam64/SmartMobility/assets/83468895/5bbfb836-a598-4451-aef6-f9c996a5a308">

    The workspace folder.

Following instructions in the shell file, you will first create a package as shown in image 1 of chapter 3.

Next, you will build the package using colcon as shown in image 2 of chapter 3.

Lastly, you will source and run the package as shown in image 3 of chapter 3.


## Chapter 4 Writing a simple publisher and subscriber (C++)

<img width="455" alt="3" src="https://github.com/Rustam64/SmartMobility/assets/83468895/a5f8db56-380d-4d13-b2db-9a28373032a9">

    Publisher node.

Firstly, create a package following the instructions. The output should match image 1 in chapter 4.

Next, you download the publisher and listener nodes using wget. The output can be seen in  image 2 in chapter 4.

Download dependencies and build the file before sourcing set-up and running the talker node.

Make sure the talker node mathces  image 3 in chapter 4. Also make sure to edit the maintainer, adding your name, email and licence.

Also add these lines to the package.xml.

<depend>rclcpp</depend>

<depend>std_msgs</depend>

In the CMakeLists.txt add the lines below:

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)

ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})


  ## Chapter 5 Writing a simple publisher and subscriber (Python)
  
<img width="737" alt="3" src="https://github.com/Rustam64/SmartMobility/assets/83468895/66ed3868-7772-4a02-a7dd-7e5276faeb75">

    Publisher and Listener.

Firstly, create a package as seen in the shell file. The output should match image 1 in chapter 5.

Next, download the listener node, the out should match image 2 in chapter 5.

Lastly, run all the set-up and dependencies and launch the talker along with the listener. The output should be simial to image 3 in chapter 5.



## Chapter 6 - Writing a simple service and client (C++)

![image](https://github.com/Rustam64/SmartMobility/assets/83468895/f56c62d7-2634-442f-9af0-9e5a5ce3d74f)

    Nodes.

Open a new terminal and ensure your ROS 2 installation is sourced.

Navigate to your ROS 2 workspace directory created in a previous tutorial (e.g., "ros2_ws").

Go into the "src" directory within your workspace since packages should be created there.

Create a new package named "cpp_srvcli" using the ament_cmake build type.

Include dependencies on "rclcpp" and "example_interfaces," which contains the .srv file.

This command generates necessary files and folders for your package.

Create the package.

Open the package.xml file for the "cpp_srvcli" package in your text editor.

Fill in the <description>, <maintainer>, and <license> tags with appropriate information.

Ensure that the necessary dependencies ("rclcpp" and "example_interfaces") are added automatically.

Inside the "ros2_ws/src/cpp_srvcli/src" directory, create a new file named "add_two_ints_server.cpp."

Paste the provided C++ code within this file.

This code defines a service node that receives two integers as a request and responds with their sum.

The "add" function processes the request and calculates the sum.
It also logs the incoming request and the response.


The "main" function initializes ROS 2, creates a node named "add_two_ints_server," and advertises a service named "add_two_ints" for this node.

It then spins the node to make the service available.

Add the following code block to the CMakeLists.txt file to create an executable named "server" for the service node:

add_executable(server src/add_two_ints_server.cpp)

To make the executable discoverable by "ros2 run," add the following lines to install the target:

install(TARGETS
    server
  DESTINATION lib/${PROJECT_NAME})
  
Inside the "ros2_ws/src/cpp_srvcli/src" directory, create a new file named "add_two_ints_client.cpp."

Paste the provided C++ code within this file.

This code defines a client node that sends a request with two integers to the service node and receives the sum as a response.

The "main" function initializes ROS 2, checks if the correct number of arguments are provided (two integers),

creates a node named "add_two_ints_client," and creates a client for the "add_two_ints" service.

It constructs a request with the provided integers and waits for the service to become available.

Once the service is available, it sends the request and waits for the response asynchronously.

Finally, it prints the received sum or an error message.

Update the CMakeLists.txt file to create an executable named "client" for the client node:

Run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building

Navigate back to the root of your workspace and build the package

Open a new terminal and source the setup files of your workspace

Start the service node and run the client node in another terminal with 2 integer arguments.


## Chapter 7 Writing a simple service and client (Python)

Open a new terminal and ensure your ROS 2 installation is sourced.

Navigate to your ROS 2 workspace directory created in a previous tutorial (e.g., "ros2_ws").

Go into the "src" directory within your workspace since packages should be created there.

Create a new package named "py_srvcli" using the ament_python build type.

Include dependencies on "rclpy" and "example_interfaces," which contains the .srv file.

This command generates necessary files and folders for your package.

Open the package.xml file for the "py_srvcli" package in your text editor.

Fill in the <description>, <maintainer>, and <license> tags with appropriate information.

Add the same information to the setup.py file for the maintainer, maintainer_email, description, and license fields.

Ensure that the necessary dependencies ("rclpy" and "example_interfaces") are added automatically.

Inside the "ros2_ws/src/py_srvcli/py_srvcli" directory, create a new file named "service_member_function.py."

Paste the provided Python code within this file.

This code defines a service node that receives two integers as a request and responds with their sum.

To allow the "ros2 run" command to run your service node, add the following line between the 'console_scripts' brackets in setup.py: 'service = py_srvcli.service_member_function:main',

Inside the "ros2_ws/src/py_srvcli/py_srvcli" directory, create a new file named "client_member_function.py."

Paste the provided Python code within this file.

This code defines a client node that sends a request with two integers to the service node and receives the sum as a response.

