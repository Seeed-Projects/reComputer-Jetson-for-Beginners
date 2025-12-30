# Use NoMachine to remotely connect to the Jetson desktop

## Introduction

​	NoMachine is a remote desktop software that allows users to securely access and control computers from anywhere. It provides fast and smooth performance by using the NX protocol, enabling tasks such as working on files, running applications, watching videos, or playing games on a remote machine as if sitting in front of it. NoMachine supports multiple platforms including Windows, macOS, Linux, and mobile devices, and is often used for remote work, technical support, and collaboration.

![image-20250926160018342](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926160018342.png)



## Installation on PC

**Step 1.**Installation Match the version of your PC system.

[Nomachine](https://www.nomachine.com/)

![image-20250926161736173](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926161736173.png)

> Note：Here we will use the ubantu pc host for the demonstration.

https://download.nomachine.com/download/?id=1&platform=linux

![image-20250926162027354](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926162027354.png)

**Step 2.** Install the client.

```bash
#INSTALL
cd /usr 
wget https://web9001.nomachine.com/download/9.1/Linux/nomachine_9.1.24_6_x86_64.tar.gz
sudo tar xvzf  nomachine_9.1.24_6_x86_64.tar.gz
rm nomachine_9.1.24_6_x86_64.tar.gz
sudo /usr/NX/nxserver --install

```

Other operations(optianal)

```bash
#UPDATE
cd /usr
sudo tar xvzf  <pkgName>_<pkgVersion>_<arch>.tar.gz 
sudo /usr/NX/nxserver --update
#UNINSTALL
sudo /usr/NX/scripts/setup/nxserver --uninstall 
sudo rm -rf /usr/NX
```

## Installation on Jetson

**Step 1.** Download and install

```bash
wget https://web9001.nomachine.com/download/9.1/Arm/nomachine_9.1.24_6_arm64.deb
sudo dpkg -i nomachine_9.1.24_6_arm64.deb
```

After installation is completed, the following key information will be displayed:

![image-20250926160530003](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926160530003.png)

If your Jetson is not connected to a display, additional configuration is required to use Remote Desktop properly.

**Step 2.** Check jetson's ip address 

![image-20250926165250083](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926165250083.png)

**Step 3.** We need to stop the system's graphics display manager and let NoMachine use the built-in virtual display service：

```bash
# 1. Stop the graphical display manager of the system
sudo systemctl disable gdm3 --now

# 2. Restart the NoMachine service
sudo /etc/NX/nxserver --restart
```

![image-20250926161036069](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926161036069.png)Jetson is not connected to a monitor. PC can display the jetson graphical interface normally by connecting a nomachine to jetson.



> Note：If you want to connect a Jetson to a monitor to display images, you need to re-enable the gdm3 service.Run the following command:

```bash
# Re-enable the gdm3 service to start automatically at boot
sudo systemctl enable gdm3
# Start the gdm3 service immediately
sudo systemctl start gdm3
```

## Connection

Add->Add connection 

![image-20250926164129169](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926164129169.png)

Enter the name and the ip address of jetson

![image-20250926164316983](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926164316983.png)

Connection

![image-20250926164514138](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926164514138.png)

Enter your jetson username and password

![image-20250926164548202](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926164548202.png)

You have successfully connected jetson. Now you can operate your jetson on your pc.

![image-20250926164651900](/home/zibochen/data/wiki/reComputer-Jetson-for-Beginners/3-Basic-Tools-and-Getting-Started/3.10-Nomachine/images/image-20250926164651900.png)

> Note:The above connection needs to be within a local area network. If the PC and jetson are not in the same local area network, the connection will fail！

