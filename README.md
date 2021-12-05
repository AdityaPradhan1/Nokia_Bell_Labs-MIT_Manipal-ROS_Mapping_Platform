# ROS_Mapping_Platform

## About the Project
The project aims to build a middleware layer connecting ROS with non ROS robots. This project acts as a building block to utilize ROS algorithms and functionalities with robots that do not natively support ROS. The main aim of this project is providing support for exploration algorithms and autonomous mapping for non ROS robots and experimenting with different environments.

### Project Structure
- The final project folder contains the final code of the project along with the presentation, documentation file, videos, as well as steps how to setup, run the code, along with the code flow.
- The implementation folder includes the various approaches and all the projects from the beginning
- The TODOList.txt contains the future tasks where this project can be extended and improved 

## Installation

### 1. ROS
The version of ROS used throughout the project was ROS Melodic on Ubuntu 18.04.5 LTS with the following script
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
```
To setup the environment
```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Dependencies for building packages
```shell
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```
Creating Workspace
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Refer http://wiki.ros.org/ documentation  

### 2. Unity
- Install Unity hub (https://unity.com/download)
- Install Unity 2020.3.14f1(lts) and 2019.4.28f1(lts) to run implementation and final project from installs in navigation bar

### 3. Rosbridge
```shell
sudo apt-get install ros-meodic-rosbridge-server
```

### 4. Gazebo
Version 9 was used in the project
```shell
 curl -ssL http://get.gazebosim.org | sh
```
or
```shell
sudo apt-get install gazebo9
sudo apt-get install libgazebo9-dev
```

### 5. rviz
```shell
sudo apt-get install ros-melodic-rviz
```
### 6. Node.js and npm
```shell
sudo apt install nodejs
sudo apt install npm
```

## Contributors
[Aditya Pradhan](https://github.com/AdityaPradhan1), 
[Arihant Jha](https://github.com/arihaaant)

### Mentors
[Manickam Elangovan](mailto:elangovan.manickam@nokia-bell-labs.com),
[Shilpa Puttegowda Pais](mailto:shilpa.puttegowda_pais@nokia.com),
[Poornalatha G](mailto:poornalatha.g@manipal.edu)
