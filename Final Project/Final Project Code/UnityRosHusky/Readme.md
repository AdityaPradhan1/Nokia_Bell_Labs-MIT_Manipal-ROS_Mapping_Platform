# ROS_Middleware_Layer
Nokia ROS Project that enables to send and receive data betweeen ROS and Unity 


## Installation and Setup
This project is intended for use with 2 different PC's or one PC running Windows and Linux (Virtual Machine).

### Prerequisites 

**Unity Installation**
Version used in this project : 2020.2.2 (Using older versions may make the project incompatible)

In the unity environment go to *'Robotics/ROS Settings'* and replace the ROS IP address with the ip address of your Ubuntu machine.

**ROS Installation**

Version used in this project : Noetic 

Create a catkin workspace and clone this repo inside it.

**Dependencies** 
slam-toolbox package
rosdep 
ros-noetic-navigation

After installing all the dependencies, build the workspace

##### Installation complete

## Running the package

Before launching the packages make sure all of them are compiled successfully.

Several Launch files are already created to launch the different nodes in this project.

***To run the reactive navigation which uses slam-toolbox***

Go to `Mobile-Robot-Navigation-and-Mapping/mobile_robot_navigation_project/config`
In the params.YAML file replace the ROS_IP with the IP of your windows machine running Unity.

Launch Unity and load the project.
To start communication between ROS and Unity 

`source /opt/ros/noetic/setup.bash`
`source ~/sofar_ws/devel/setup.bash`
`roslaunch mobile_robot_navigation_project navigation.launch`

Hit the play button in Unity. Communication should be established. Unity should show the sending and receiving of data.
You can echo the various topics being published to the terminal to verify if data is being exchanged bewtween Unity and ROS.

***To start robot navigation towards goal point***

While the project and launch files are runnnig run another launch files for the remaining nodes.

`source /opt/ros/noetic/setup.bash`
`source ~/sofar_ws/devel/setup.bash`
`roslaunch mobile_robot_navigation_project mapless_nav.launch`

After all the nodes are running successfully use the *"Move!"* (top right) button in Unity and this would send a goal message to ROS containing the pose of the target.

The robot should navigate through the Unity environment with the help of slam-toolbox and reach goal destination.
You can echo different topics being exchanged between ROS and Unity. The pose and command velocity (cmd_vel) are some topics you can echo to verify working of slam.

***To save the map after Robot has navigated through the environment***

The map generated can be saved using the following launch file

`source /opt/ros/noetic/setup.bash`
`source ~/sofar_ws/devel/setup.bash`
`rosrun map_server map_saver -f map`

This will save a map file inside ROS which can further be used when navigating again in the same environment to move faster and require less computation power.


## Map Based Navigation 

Initial setup of the project remains same.

In this architecture the robot has no idea of its initial location and hence needs to be provided with that for effective navigation.
Use the *2D Pose Estimate* button in rviz to set the location of the robot initially corresponding to Unity environment.

This navigation is faster and easier since the robot already has a map of the environment.
AMCL helps the robot localise itself, with respect to the map.

To run the mapbased navigation, close all the previous launch files and run

`source /opt/ros/noetic/setup.bash`
`source ~/sofar_ws/devel/setup.bash`
`roslaunch mobile_robot_navigation_project mapbased_nav.launch`


### Map Based Navigation

Better for repetitive task where the map doesn't change often.

### Reactive based Navigation

Better suited for unknown or dynamic environment.





