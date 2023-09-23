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


