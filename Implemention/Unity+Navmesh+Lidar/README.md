# Nokia_Bell_Labs-MIT_Manipal-ROS_Mapping_Platform

Copy robot_ros_unity package in the catkin_ws(or alternate ros package) and build or catkin make it
Open navmesh lidar testing in unity (v2019)

Install rosbridge if not there
sudo apt-get install ros-melodic-rosbridge-server

## Setup and run
1. source ~/catkin_ws/devel/setup.bash
   open new terminal for each command below
2. roscore
3. roslaunch rosbridge_server rosbridge_websocket.launch
4. cd src/robot_ros_unity/scripts/
   ./scan.py
5. cd src/robot_ros_unity/scripts/
   ./map.py
6. rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 /base_link /base_scan
7. rosrun gmapping slam_gmapping _linearUpdate:=0.0 _angularUpdate:=0.0
8. rosrun rviz rviz
Add topics like map , laserscan, etc
