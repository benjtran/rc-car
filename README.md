# Mobile Robot

A 4WD mobile robot buggy running SLAM for real time motion planning.

<figure>
    <img src="images\isometric.jpg" width="250" alt="Iris Plot">
    <figcaption>Figure 1: Isometric View of Robot</figcaption>
</figure>

<figure>
    <img src="images\demo.gif" width="250" alt="Iris Plot">
    <figcaption>Figure 2: Demo of Robot in Random Environment</figcaption>
</figure>

Note: This project was supposed to be an RC car, hence the repo name. I reworked it to see if I could make it autonomous!

## Hardware

### Mechanical:

The chassis and body were custom-designed in Autodesk Inventor. The wheels were sourced from Bambu Studio. All components were 3D printed with PLA.

<figure>
    <img src="images\cad.PNG" width="250" alt="Iris Plot">
    <figcaption>Figure 3: Isometric View of Chassis and Body</figcaption>
</figure>

### Electrical:
- [1 x Raspberry Pi 5](https://www.amazon.ca/Raspberry-Pi-8GB-2023-Processor/dp/B0CK2FCG1K/ref=sr_1_6?dib=eyJ2IjoiMSJ9.vCrg1_s9-d710VGX_QZKTJpWoieDSCeytda8d4WgxFw4Gid8tfJ6FYDPM6bZXHUV9XDREaLVwtqJYC03ABK790MiU_2eJVu5E1WtiEgN2AIbuh0CLdAU4Kafkepuhyod41wYRrKKt9b8WV652Cxy_Fk1vsw023WgwV9zADHfGq-VZy6jEE7X7QBAKrgD5aX9G4iRTcy7zw50pzEPDVVYK6oqHuJjG0ONkuMOi7H-_IxVX-QKHj7c2AGP-R1W6Ak5H41mHIDRrMUVRNAh-VyhS3NVct9yYLmwixyMsCuFGuw.rcXRIKtaj8TOREFSbxQ-Y5d63ZMBOAlZG9mNsSeW_DI&dib_tag=se&keywords=raspberry+pi+5&qid=1769126089&sr=8-6)
- [1 x Arduino Mega 2560](https://www.amazon.ca/ELEGOO-Compatible-Arduino-Projects-Compliant/dp/B01H4ZLZLQ/ref=sr_1_1_sspa?crid=1EE6USV8UHDRO&dib=eyJ2IjoiMSJ9.C9li7QlUOdnawgCr8xZlUWCAPTATchOLaEpEWIsdc-4e6Cunn8ds_Bv8XgCd51L1QQlFOne0i1lbvro5i4ALs9cwBDXU5egJE0kntgR4RArPOwW3BdiUog5MFsodNSBjc4GzoAByxMYDSK_U7elLz2nyYBK-Bva8cBZXPnZX7ktnP7qdxs_RYB-PZVPy69IPsziNi-pWZMVzGEpKxnJbIY0EqL7MCpqLBsN1pnWzs33YlLP_IISOmw3xN6TZL-Wio-ffk6kqIJLG7U4bxT8qfxw8c2_JdfsCxCP1ghYFeKA.d1AxuKDTNtAvrDDgq1wpfIM5io7lRFapAMFB62D1ga4&dib_tag=se&keywords=arduino%2Bmega&qid=1769125918&sprefix=arduino%2Bmeg%2Caps%2C110&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1)
- [1 x 12V 5600mAh Lithium Ion Battery](https://www.amazon.ca/Mspalocell-Rechargeable-Battery-Compatible-Electronic/dp/B0D6R4RK26/ref=sr_1_10?crid=2ZGRN048L4YRN&dib=eyJ2IjoiMSJ9.SJ30AE5_2wN8P1ufM9TpjvY6X4JUTYwxt1G0HYHwIhBxa8RcnxsgCocR-Go3oNw9EhE4HnxA1U4ZhWmzyp00EB_sJNxN-FHQyd4AxuR-AJnqF6GjaGidvfwKMl5yCMyplsSd175hVjZWIJIt5ATARD7IZHvM-_mXLHI4TL8ee4Wm2l89-01qXBo15J2xDA5BYc08F50D5UfXVjPjQKH6KyRSKtI1bX7tHTOTjRgngDwMd-LGNARu6-sQsDx6W0ZCH2l1tzyIVE2ZifoWi4tIT8_Cn3j3IdyD8WIfCPENvy4.49jeO6sOM5saQCuNBwrMbKvB4034mFJeR7mZvj8RSw4&dib_tag=se&keywords=rechargeable+battery+12v&qid=1769126127&sprefix=rechargable+battery+12v%2Caps%2C132&sr=8-10)
- [1 x GreekPi Power Expansion Board](https://www.amazon.ca/GeeekPi-Expansion-Raspberry-Automatic-Function/dp/B0CYPRDY9Q/ref=sr_1_52?crid=2MNSSA2FMUAFK&dib=eyJ2IjoiMSJ9.FBn7yZcB3xol--OFYTsfG9bi08MpdAq9G79-yfBx_McYGEaasC6y4E7swuW1g8dpwE87pNKqwX0zOttfH2aZTa0vqTaqvEKkIspYPjjXKS0hBYMXUhJohcHMnF3I9Z6ai0cUXiR4ZgfyoqFPYSF9uoHDhwzUgnm3ZQBxlmMk_DO8AnlOj_O1gcVFoMQ00y4OMWnjXIwQfXoZZt5DNMzB1wvBW6gMom7L7IFtUQAq-F-2D9wI5_uaUt_FzwUyO7MG5H4IjZRZkcI-69fSbc-P1JSws1-0T5gxX8EWg6Zn8YE.Ce6LWEqtJgaHYa4MiTn9O3KybYBGAD6ck4JbbV9njLo&dib_tag=se&keywords=raspberry+pi+5+power+module&qid=1769126846&sprefix=raspberry+pi+5+power+module%2Caps%2C90&sr=8-52&xpid=UriyH_tSCW3ws)
- [2 x L298N Motor Driver](https://www.amazon.ca/BOJACK-H-Bridge-Controller-Intelligent-Mega2560/dp/B0C5JCF5RS/ref=sr_1_1_sspa?crid=2LK80KC227HER&dib=eyJ2IjoiMSJ9.7-M0lWw-lMXXgJGoLjEzsUVZJ_RBhdA8a-KYaHEbNt9_Ua9O9rH3RWXHUs-VEfR0WPqWLJsg9HTDAFwMnCKFoBWu3UE9TYo9mzYJrVZ8DnXLjUwqW7XrjNrDn13DRy29XgDshRo3jZzdaMhjC6ZUmXarUxbE_RuNmiHRhJvS4-FM2_lzRI4Whxl7N7b7KxJnSyCfTSf5ca3GrI7RP9zqyxloCmuPdl62YmnDXlQRy2UH25jRDE583elmcwE7zjPSXLjOTFEEvIzVFs3psqXLnAVGlyYE--NtDVm91uV81eI.CmM972tHFbQRlX8XO4H4lYczBJN9Q2qdfrem3xRjeKo&dib_tag=se&keywords=l298n+motor+driver&qid=1769125842&sprefix=L2%2Caps%2C120&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1)
- [4 x 12V DC Geared Motor with Encoder](https://www.amazon.ca/Encoder-Mounting-Bracket-Magnetic-Reduction/dp/B07WT2TCPZ/ref=pd_ci_mcx_di_int_sccai_cn_d_sccl_2_1/131-1014396-9326462?pd_rd_w=pGOwj&content-id=amzn1.sym.d6674fdf-bd00-4d07-8317-6dfd6c498cdf&pf_rd_p=d6674fdf-bd00-4d07-8317-6dfd6c498cdf&pf_rd_r=4QY3NH0PKRBKPYSXT1ZH&pd_rd_wg=JtXds&pd_rd_r=4441beb9-7887-4fe5-975f-f7619fa47712&pd_rd_i=B07WT2TCPZ&psc=1)
- [1 x RPLIDAR C1](https://www.amazon.ca/SLAMTEC-Navigation-Obstacle-Avoidance-Interface/dp/B0FRM8KHZF/ref=sr_1_6?crid=1FGDC7LUFO3F3&dib=eyJ2IjoiMSJ9.UXT0uzkLVHW5l-1ud3DVgzcl8fBRfBUdMjMdYar-UAvkcCKkry7veX6o4LdgyOXskV0ChTE4MlJRhvUrMsbZJJ1MyagYUhey843Yf_yep7mUsZ8aiosYSRziKwiYLsp0Inezga-wSjdmZcE7v4qdiQyp6fAWtVk2-HXlVTRhrCPh0VBBChIWLtCRd-SygVktUVHswc3_9XnxuXw_jfaWueUKMGPvnjHoKNZAGlqJ1FAeUAZ3SZ0i0VYQextEBnfDfSphp85OznKqY4Vx83a6fuXnQI9MoMGnQDRhngxYpDs.ibpnD7hFYqa4CaaVRv3qB6ZArPVsJZ8zJEOfpj-kCi8&dib_tag=se&keywords=rplidar&qid=1769126736&sprefix=rplida%2Caps%2C110&sr=8-6)

## Software

1. Initialization: The Raspberry Pi launches the ROS network.

2. Sensing: The RPLIDAR node publishes raw 2D laser scan data (/scan) of the room.

3. SLAM: The Hector SLAM node listens for /scan to simultaneously build a 2D Occupancy Grid (/map) and estimate the robot's location (/tf).

4. Costmap: Path Planner node converts the map into a Costmap, assigning a cost to each cell of the grid indicating danger levels (cells close to walls are dangerous).

5. Frontier Selection: Path Planner node identifies frontiers (boundaries between free and unknown space), filters out blind spots on lidar, and selects the closest reachable target.

6. Pathfinding: Path Planner node generates a path to the target using A*, prioritizing paths through the center of open spaces rather than hugging dangerous areas.

7. Path Following: Path Planner node uses an incremental controller to drive the robot waypoint-by-waypoint, adjusting velocity and heading to follow the trajectory.

8. Active Safety Check: Path Planner node continuously traces the path ahead; if an obstacle appears or the robot drifts unexpectedly, it halts immediately to replan.

9. Emergency Escape: Path Planner Node checks entering a high-danger zone (too close to a wall), a high-priority "Escape" behavior overrides the planner to back the robot away.

10. Re-planning: Path Planner Node monitors the target frontier; if the target area is completely mapped before arrival, it cancels the current path and selects a new frontier.

11. Velocity Commands: Path Planner Node computes required velocity for each motor and publishes to /cmd_vel.

12. Motor Interface: Motor Interface Node listens for /cmd_vel and sends it to Arduino through serial.

13. Motor Controller: Arduino recieves target velocities and controls motor speeds using a PID controller.

11. Completion: The loop repeats until no accessible frontiers remain, at which point the robot stops.

## Setup Instructions (for me)

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
cd ~/rc-car/catkin_ws
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
cd ~/rc-car/catkin_ws
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
cd ~/rc-car/catkin_ws
source devel/setup.bash
rosrun robot_control motor_interface_node.py 
```
8. Enter fifth container:
```bash
sudo docker exec -it hector_ros bash 
```
9. Start path planning:
```bash
source /opt/ros/noetic/setup.bash
cd ~/rc-car/catkin_ws
source devel/setup.bash
rosrun robot_navigation path_planner_node.py 
```