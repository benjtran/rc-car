# Mobile Robot

A 4WD mobile robot buggy running SLAM for real time motion planning.

## Instructions

1. Run GUI enabled docker container:
```bash
xhost +local:docker
sudo docker start hector_ros
sudo docker exec -it hector_ros bash
```
2. Start ROS1:
```bash
source /opt/ros/noetic/setup.bash
roscore
```
3. Enter second container:
```bash
sudo docker exec -it hector_ros bash
```
4. Launch rplidar:
```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch rplidar_ros view_rplidar_c1.launch
```
5. Enter third container:
```bash
sudo docker exec -it hector_ros bash 
```
6. Launch SLAM:
```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch hector_slam_launch tutorial.launch
```

6. Enter fourth container:
```bash
sudo docker exec -it hector_ros bash 
```
7. Start motor interface:
```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch robot_control control.launch
```
