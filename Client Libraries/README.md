# ROS 2 Tutorials

Welcome to the ROS 2 tutorials! This guide will walk you through various topics related to ROS 2 development. Below is an index of the chapters covered in this tutorial:

1. [Using Colcon to Build Packages](#chapter-1-using-colcon-to-build-packages)
2. [Creating a Workspace](#chapter-2-creating-a-workspace)
3. [Creating a Package](#chapter-3-creating-a-package)
4. [Writing a Simple Publisher and Subscriber (C++)](#chapter-4-writing-a-simple-publisher-and-subscriber-c)
5. [Writing a Simple Publisher and Subscriber (Python)](#chapter-5-writing-a-simple-publisher-and-subscriber-python)
6. [Writing a Simple Service and Client (C++)](#chapter-6-writing-a-simple-service-and-client-c)
7. [Writing a Simple Service and Client (Python)](#chapter-7-writing-a-simple-service-and-client-python)
8. [Creating Custom Msg and Srv Files](#chapter-8-creating-custom-msg-and-srv-files)
9. [Implementing Custom Interfaces](#chapter-9-implementing-custom-interfaces)
10. [Using Parameters in a Class (C++)](#chapter-10-using-parameters-in-a-class-c)
11. [Using Parameters in a Class (Python)](#chapter-11-using-parameters-in-a-class-python)
12. [Using ros2doctor to Identify Issues](#chapter-12-using-ros2doctor-to-identify-issues)
13. [Chapter 13 - Creating and Using Plugins (C++)](#chapter-13-creating-and-using-plugins-c)

Feel free to jump to a specific chapter that interests you or follow along from the beginning. Happy learning!


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
    
In this chapter, you'll learn how to create a simple service and client in ROS 2 using C++. The example involves creating a package, building it, and running a service and client node to add two integers.

### Instructions:

1. Create a new ROS 2 package named "cpp_srvcli" with dependencies on "rclcpp" and "example_interfaces" using the following command:
   ```
   ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
   ```

2. Use `rosdep` to install dependencies for the package from the `src` directory:
   ```
   rosdep install -i --from-path src --rosdistro foxy -y
   ```

3. Build the "cpp_srvcli" package using `colcon build --packages-select cpp_srvcli`.

4. Source the setup files to set up the ROS 2 environment:
   ```
   source install/setup.bash
   ```

5. Start the service node:
   ```
   ros2 run cpp_srvcli server
   ```

6. Run the client node with two integers as arguments to request addition:
   ```
   ros2 run cpp_srvcli client 2 3
   ```

## Chapter 7 - Writing a Simple Service and Client (Python)

![image](https://github.com/Rustam64/SmartMobility/assets/83468895/36281197-c5d8-483c-99d5-0ed46c06d89d)

    Publisher and listener nodes.

This chapter covers creating a simple service and client in ROS 2 using Python. You will create a package, define a service server, and write a client that communicates with the server to add two integers.

### Service Server (Python):

1. Create a new ROS 2 package named "py_srvcli" with dependencies on "rclpy" and "example_interfaces."

2. Implement a service server node (`MinimalService`) that adds two integers when a request is received.

### Client (Python):

3. Create a client node (`MinimalClientAsync`) that sends a request to the service server and receives a response.

4. Source the setup files to set up the ROS 2 environment.

5. Run the service node:
   ```
   ros2 run py_srvcli service
   ```

6. Open another terminal, source the setup files again, and start the client node by providing two integers as arguments:
   ```
   ros2 run py_srvcli client 2 3
   ```

7. The client node will send the request, and you will receive the result of the addition:
   ```
   [INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
   ```

## Additional Notes for Python Service/Client:

8. To enable running the client node using `ros2 run`, add the following line to the 'console_scripts' section in `setup.py`:
   ```
   'client = py_srvcli.client_member_function:main',
   ```

9. Before building, run `rosdep` in the root of your workspace ("ros2_ws") to check for missing dependencies:
   ```
   rosdep install -i --from-path src --rosdistro foxy -y
   ```

10. Build the package in your workspace using `colcon build --packages-select py_srvcli`.

11. Open a new terminal, navigate to your workspace ("ros2_ws"), and source the setup files:
    ```
    source install/setup.bash
    ```

12. Start the service node:
    ```
    ros2 run py_srvcli service
    ```

13. In another terminal, you can run the client node with two integer arguments, as demonstrated above.

## Chapter 8 - Creating Custom Msg and Srv Files

This chapter demonstrates how to create custom message (msg) and service (srv) files in ROS 2. It also covers generating language-specific code and modifying ROS 2 packages to use these custom interfaces.

### Instructions:

1. Create a new package named "tutorial_interfaces" in your ROS 2 workspace using the following command:
   ```
   ros2 pkg create --build-type ament_cmake tutorial_interfaces
   ```

2. Inside the `tutorial_interfaces/msg` directory, create a new file called `Num.msg`. Define a custom message that transfers a single 64-bit integer called "num."

3. Inside the `tutorial_interfaces/msg` directory, create a new file called `Sphere.msg`. Define a custom message that includes a message from another package (`geometry_msgs/Point`) and a float64 called "radius."

4. Inside the `tutorial_interfaces/srv` directory, create a new file called `AddThreeInts.srv`. Define a custom service with a request containing three int64 fields (a, b, c) and a response containing a single int64 field called "sum."

5. Modify the `CMakeLists.txt` file to find the `geometry_msgs` package and add code to generate interfaces for your custom messages and services.

6. Modify the `package.xml` file to declare dependencies and build and execution dependencies.

7. Build the "tutorial_interfaces" package to generate language-specific code using `colcon build`.

8. Source the workspace to make the custom interfaces discoverable using `source install/setup.bash`.

9. Check the message definitions using the 'ros2 interface show' command for `tutorial_interfaces/msg/Num`, `tutorial_interfaces/msg/Sphere`, and `tutorial_interfaces/srv/AddThreeInts`.

10. Modify the publisher and subscriber nodes to use the custom `Num.msg` for publishing and receiving integer values. Add dependencies on `tutorial_interfaces` for both publisher and subscriber.

11. Modify the client and server nodes to use the custom `AddThreeInts.srv` for requesting and responding with integers. Add dependencies on `tutorial_interfaces` for both server and client.

12. Build the package that includes publisher, subscriber, client, and server nodes using `colcon build --packages-select <package_name>`.

13. Run the nodes:
    - Publisher: `ros2 run <package_name> talker`
    - Subscriber: `ros2 run <package_name> listener`
    - Client and Server: 
      - Server: `ros2 run <package_name> server`
      - Client: `ros2 run <package_name> client <arg1> <arg2> <arg3>`

## Chapter 9 - Implementing Custom Interfaces

![image](https://github.com/Rustam64/SmartMobility/assets/83468895/8335d6b3-6360-4357-9af0-6c91eafd8e8a)

    Address book.


In this chapter, you'll learn how to create a custom message file and use it in a ROS 2 package. The example involves creating a package named "more_interfaces" and implementing a custom message file.

### Instructions:

1. Create a new package named "more_interfaces" with the message directory using the command:
   ```
   ros2 pkg create --build-type ament_cmake more_interfaces
   mkdir more_interfaces/msg
   ```

2. Create a custom message file named `AddressBook.msg` inside the `more_interfaces/msg` directory. Define fields for a custom message.

3. Update the `package.xml` file to specify build and runtime dependencies.

4. Update the `CMakeLists.txt` file to generate interfaces for your custom message.

5. Create a node using the custom message and build it.

6. Source the workspace and run the publisher node.

## Chapter 10 - Using Parameters in a Class (C++)

![image](https://github.com/Rustam64/SmartMobility/assets/83468895/fe388b6f-6f55-46eb-8039-51a81ed005b7)

    Minimal parameter node.


This chapter guides you through creating a ROS 2 workspace and writing a C++ node that uses parameters. You will also learn how to change parameters via the console and launch files.

### Instructions:

1. Create a new ROS 2 workspace named "ros2_ws."

2. Create a package named "cpp_parameters" with dependencies on "rclcpp."

3. Write a C++ node that declares a parameter, updates it in a timer callback, and builds the package.

4. Source the setup files to set up the ROS 2 environment.

5. Run the C++ node, change parameters via the console, and list parameters.

6. Change parameters via a launch file.

## Chapter 11 - Using Parameters in a Class (Python)

This chapter demonstrates how to use parameters in a Python-based ROS 2 node. You will create a package, write a Python node that declares and updates parameters, and change parameters via the console and launch files.

### Instructions:

1. Create a new ROS 2 workspace named "ros2_ws."

2. Create a package named "python_parameters" with dependencies on "rclpy."

3. Write a Python node that declares a parameter, updates it in a timer callback, and builds the package.

4. Source the setup files to set up the ROS 2 environment.

5. Run the Python node, change parameters via the console, and list parameters.

6. Change parameters via a launch file.

## Chapter 12 - Using ros2doctor to Identify Issues

This chapter covers using the `ros2doctor` tool to diagnose and report issues in your ROS 2 setup. It includes checking ROS 2 setup, launching nodes, and generating a detailed report.

### Instructions:

1. Run `ros2 doctor` to check

 the ROS 2 setup.

2. Start the turtlesim system and teleop controls in separate terminals.

3. Run `ros2 doctor` again and check for system status.

4. Create subscribers for topics to generate data.

5. Run `ros2 doctor --report` to get a full report of your ROS 2 setup.

## Chapter 13 - Creating and Using Plugins (C++)

This chapter focuses on creating and using plugins in ROS 2 using C++. It involves creating a base package, defining a base class, creating plugin packages, and using the plugins in a ROS 2 node.

### Instructions:

1. Create a package named "polygon_base" to define the base class for plugins.

2. Define the `RegularPolygon` base class with virtual methods for initialization and area calculation.

3. Create a package named "polygon_plugins" to implement Square and Triangle plugin classes.

4. Implement the Square and Triangle classes that inherit from the `RegularPolygon` base class.

5. Create a `plugins.xml` file to declare the plugins.

6. Export the plugin classes using `PLUGINLIB_EXPORT_CLASS`.

7. Build the packages using `colcon build`.

8. Source the setup files to set up the ROS 2 environment.

9. Create a ROS 2 node that loads and uses the Square and Triangle plugins.
