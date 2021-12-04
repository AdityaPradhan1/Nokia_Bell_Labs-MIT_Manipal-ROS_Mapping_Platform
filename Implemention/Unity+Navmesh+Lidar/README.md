# Nokia_Bell_Labs-MIT_Manipal-ROS_Mapping_Platform

Copy robot_ros_unity package in the catkin_ws(or alternate ros package) and build or catkin make it  
Open navmesh lidar testing in unity (v2019)  

Install rosbridge if not there  
    `sudo apt-get install ros-melodic-rosbridge-server`

## Setup and run
1. ```shell
	source ~/catkin_ws/devel/setup.bash
	```
   open new terminal for each command below 
2. ```shell
	roscore
	```
4. ```shell
	roslaunch rosbridge_server rosbridge_websocket.launch
	```
5. ```shell
	cd src/robot_ros_unity/scripts/ 
	./scan.py
   ```
5. ```shell
	cd src/robot_ros_unity/scripts/`  
   ./map.py
   ```
6. ```shell
	rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 /base_link /base_scan
	```
8. ```shell
	rosrun gmapping slam_gmapping _linearUpdate:=0.0 _angularUpdate:=0.0
	```
10. ```shell
	rosrun rviz rviz
	```
Add topics like map , laserscan, etc
