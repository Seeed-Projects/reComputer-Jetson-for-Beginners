# Single-Line-Lidar A1

A single-line LiDAR with one end connected to a micro USB can be powered and communicate directly with the reComputer. As shown in the figure below:

![Imgur](https://i.imgur.com/llr4VYI.jpg)

We have provided the SDK for the single-line LiDAR. Of course, you can also download it from [here](https://github.com/robopeak/rplidar_ros).

A1 ROS SDK is a wrapper for RoboPeak SDK that supports ROS Kinetic, Melodic, and Noetic distributions.

RPLIDAR ROS package
===================

ROS node and test application for RPLIDAR

Visit following Website for more details about RPLIDAR:

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage:   http://www.slamtec.com/en/Lidar

rplidar SDK: https://github.com/Slamtec/rplidar_sdk

rplidar Tutorial:  https://github.com/robopeak/rplidar_ros/wiki

How to build rplidar ros package
================================

```bash
cd Single-line-lidar_A1_ws
catkin_make
source devel/setup.bash
```

How to run rplidar ros package
==============================

Type the following in the terminal:

```bash
ls /dev/ttyUSB*
```

Then the terminal will print the serial port number mapped to the single-line LiDAR, typically as `/dev/ttyUSBx` **(x = 0, 1, 2, 3...)**.

Open the `rplidar.launch` file in the `launch` folder, and replace the highlighted content in the figure with your serial port number, which could be one of 0, 1, 2, 3, etc.

![Imgur](https://i.imgur.com/3Udiu2F.png)

After completing these tasks.There're two ways to run rplidar ros package

I. Run rplidar node and view in the rviz
----------------------------------------

For RPLIDAR A1/A2

```bash
roslaunch rplidar_ros view_rplidar.launch
```

You should see rplidar's scan result in the rviz.
![Imgur](https://i.imgur.com/X2VV3zu.png)

II. Run rplidar node and view using test application
----------------------------------------------------

roslaunch rplidar_ros rplidar.launch (for RPLIDAR A1/A2)
or
roslaunch rplidar_ros rplidar_a3.launch (for RPLIDAR A3)

rosrun rplidar_ros rplidarNodeClient

You should see rplidar's scan result in the console

Notice: the different is serial_baudrate between A1/A2 and A3

RPLidar frame
=============

RPLidar frame must be broadcasted according to picture shown in rplidar-frame.png


---

**If you want to know about related products, you can click the link below:**

[ReComputer J1020 v2 nano. ](https://www.seeedstudio.com/reComputer-J1020-v2-p-5498.html)

[Ros robot kit. ](https://www.aliexpress.us/item/3256801169020544.html?gatewayAdapt=glo2usa)

---

