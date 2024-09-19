# Basic motion control methods

### Introduction

This section will introduce how to perform simple motion control of the robot chassis using APP, PS2 controller, or keyboard. All three control methods transmit control commands via Bluetooth.

![]()

---

### Use APP to control robots

##### Obtain Bluetooth APP

For users with Android phones, you can download the documentation we provide to obtain the Bluetooth app. For example, for the ROS robot model <mark>XXX</mark>, the app file name is as follows *WHEELTEC_1.1.5.apk* 

For users with iPhones, you only need to search for WHEELTEC in the App Store to download and use it.

##### Bluetooth connection to robot

1. **Power the robot**

       After turning on the robot's power switch, you will see the red indicator light of the        Bluetooth module blinking, indicating that the Bluetooth module is in               an unconnected state.

<img title="" src="https://i.imgur.com/dB5bSsv.jpg" alt="Imgur" data-align="center" width="493">

2. **Open Bluetooth app**
   
   Open the Bluetooth app installed in the *"Obtain Bluetooth APP"*, and you will see an interface like the one shown below:
   
   <img title="" src="https://i.imgur.com/pCNYOBc.jpg" alt="Imgur" data-align="center" width="347">

3. **Connect Bluetooth module**
   
   Open the following interface by clicking the button consisting of three horizontal bars in the upper left corner, then search for the Bluetooth named BT-04A and connect. The connected pairing code is 1234. After the connection is successful, the Bluetooth module indicator light becomes solid.
   
   <img title="" src="https://i.imgur.com/wRiihi7.jpg" alt="Imgur" data-align="center" width="526">

---

### APP usage instructions

Introduction to the functions of the APP homepage:

<img title="" src="https://i.imgur.com/ch4iZoH.jpg" alt="Imgur" data-align="center" width="372">

| Number | Name                   | Function Description                                                                                                                                                                                                                                       |
| ------ | ---------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ①      | APP Joystick           | Used to control the robot's movement. (For details, refer to Chapter 3)                                                                                                                                                                                    |
| ②      | Control Mode           | Used to switch the robot's movement control mode.                                                                                                                                                                                                          |
| ③      | Angle Information Bar  | Used to display the Z-axis speed information of the robot.                                                                                                                                                                                                 |
| ④      | Debug Bar              | Used to display the real-time information received by the APP and the sent signals.                                                                                                                                                                        |
| ⑤      | Left and Right Encoder | Used to display the speed information of motors A and B of the robot. The left and right encoders correspond to motors A and B, respectively.                                                                                                              |
| ⑥      | Battery Progress Bar   | Used to display the robot's battery level in percentage form. For some models using lithium iron phosphate batteries, the voltage and capacity do not have a linear relationship, so 0% to 100% corresponds to a battery voltage of 20V-25.2V or 10-12.6V. |
| ⑦/⑧    | Speed Control Buttons  | Adjust the robot's movement speed. Each click increases the speed by 0.1m/s; reducing speed is similar.                                                                                                                                                    |

After connecting the Bluetooth APP to the robot's Bluetooth module, the robot's control mode will not switch immediately. The control mode of the robot will be displayed in real-time in the lower-left corner of the OLED screen on the robot.

Once the connection is established, lightly push the control joystick forward on the main page of the APP to switch the robot's control mode to APP control mode. After switching to the APP control mode, you will be able to control the robot, view waveforms, and adjust parameters using the APP.
