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

  
