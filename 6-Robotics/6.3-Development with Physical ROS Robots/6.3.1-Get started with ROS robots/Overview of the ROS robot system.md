# Overview of the ROS robot system

### Introduction

This section will introduce the system architecture of the ROS robot and provide a list of the hardware components. The ROS robot system is primarily composed of two main parts:

1. The upper-level component of perception and decision.
2. The lower-level component of motion control.

---

### The upper-level component of perception and decision

This part requires high computational power but has low real-time requirements. It is typically run on a general-purpose operating system on high-performance devices. Programs are written to perform perception and decision-making functions based on input data from various types of sensors.

**The programs for the perception and decision-making functions of the ROS robot are executed by** [reComputer J1020 v2 nano. ](https://www.seeedstudio.com/reComputer-J1020-v2-p-5498.html)

![](https://media-cdn.seeedstudio.com/media/catalog/product/cache/bb49d3ec4ee05b6f018e93f896b8a25d/1/3/13_1.jpg)
The role of the reComputer is to gather data from various sensors, process and analyze the data as needed, and then control (decide) the robot's actions, such as movement and grasping.

For example, to enable the robot to follow a red object, the camera sensor first captures the environmental image information. The reComputer processes the image to identify the position of the red object and then directs the robot to approach the object.

The reComputer can be considered a computer capable of running ROS. Since it needs to be installed inside the robot, the computer must be relatively compact.

**The input data for the perception and decision-making programs are provided by various types of sensors, including ：**

* **Motor encoders**
  
  ![Imgur](https://i.imgur.com/ZTFAdos.png)
  (Obtain the robot's motion information)
* **IMU&GNSS modules**
  
  ![Imgur](https://i.imgur.com/PxTweDJ.png)
  (Obtain external motion information)
* **Single-line LiDAR**

![Imgur](https://i.imgur.com/MvzQCpB.png)

(Obtain a two-dimensional laser point cloud)

* **Multi-line LiDAR**
  ![](https://www.livoxtech.com/dps/2d9e037e6d457ef7ffec037f7d16dcf8.png)
  (Obtain three-dimensional laser point cloud)
  
* **Depth camras**
  ![Imgur](https://i.imgur.com/Dtz1Ins.png)
  (Obtain depth-informed images)

---

### The lower-level component of motion control

This part has low computational power requirements but high real-time requirements. The programs for motion control  are written in C and run on a microcontroller. Their primary role is to execute the motion control commands issued by the upper-level system and provide feedback on the robot's current state.

**The list of hardware used for motion control is as follows:**

* **Power Battery**
  ![Imgur](https://i.imgur.com/YIfOwfY.png)
  
  (Provide energy for ROS robots)
* **Controller and Driver**
  ![Imgur](https://i.imgur.com/38YXWXR.png)
  (The controller generates control signals, and the driver amplifies these signals to drive the motor. They can be designed as an integrated unit)
* **Motor and Servo**
  ![Imgur](https://i.imgur.com/3oiYCap.png)
  (Devices that convert electrical energy into kinetic energy)
* **Chassis**
  
  ![Imgur](https://i.imgur.com/3NgbrXq.png)
  
  (The above four components are mounted on a frame to form a  chassis)

