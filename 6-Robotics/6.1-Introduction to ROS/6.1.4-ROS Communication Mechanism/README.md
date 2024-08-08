# 6.1.4-ROS Communication Mechanism: 

## Topic Communication Tutorial

### Introduction to Topic Communication

In ROS, topic communication is one of the fundamental ways for nodes to exchange information. This tutorial will guide you through the process of setting up basic topic communication using both C++ and Python. We will implement a simple publisher-subscriber model where the publisher sends text messages at a frequency of 10Hz, and the subscriber receives and prints these messages.

#### 1. Theoretical Model

Topic communication involves three main components:
-  **ROS Master**: Manages the registration and connection of nodes.
-  **Talker** (Publisher): Sends messages.
-  **Listener** (Subscriber): Receives messages.

The ROS Master helps establish connections between Talkers and Listeners. Here's a step-by-step breakdown of how the communication happens:

-  **Talker Registration**: The Talker registers itself with the ROS Master, including the topic name of its messages.
-  **Listener Registration**: The Listener registers itself with the ROS Master, specifying the topic it wants to subscribe to.
-  **Matching**: The ROS Master matches the Talker and Listener based on the topic name and sends the necessary connection information.
-  **Connection Establishment**: The Listener requests a connection to the Talker, and the Talker confirms it.
-  **Message Exchange**: Once connected, the Talker starts sending messages to the Listener.

**Key Points**:
- The ROS Master is only needed for establishing the connection.
- The communication continues even if the ROS Master is shut down after the connection is established.
- Multiple Talkers and Listeners can exist, and they can start in any order.

#### 2. Basic Topic Communication Operations (C++)

**Objective**: Create a publisher node that sends text messages at 10Hz and a subscriber node that prints the received messages.

**Steps**:

0. **[Create package](../6.1.2-Quick%20Experience%20with%20HelloWorld%20for%20ROS/README.md)**
    ```bash
    cd ~/seeed_ws/src/
    catkin_create_pkg listener_and_talker roscpp rospy std_msgs
    cd ~/seeed_ws/src/listener_and_talker/src
    touch listener.cpp talker.cpp
    ```

1. **Publisher Implementation**:
  
    `talker.cpp`
   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"
   #include <sstream>

   int main(int argc, char *argv[]) {
       // Set locale for printing messages in the local language
       setlocale(LC_ALL, "");
       // Initialize the ROS node with a unique name
       ros::init(argc, argv, "talker");
       // Create a ROS node handle
       ros::NodeHandle nh;
       // Create a publisher object
       ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);

       std_msgs::String msg;
       std::string msg_front = "Hello Seeed";
       int count = 0;
       ros::Rate r(10); // 10Hz

       while (ros::ok()) {
           std::stringstream ss;
           ss << msg_front << count;
           msg.data = ss.str();
           pub.publish(msg);
           ROS_INFO("Sent message: %s", msg.data.c_str());
           r.sleep();
           count++;
       }
       return 0;
   }
   ```

2. **Subscriber Implementation**:

    `listener.cpp`
   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"

   void doMsg(const std_msgs::String::ConstPtr& msg_p) {
       ROS_INFO("Heard: %s", msg_p->data.c_str());
   }

   int main(int argc, char *argv[]) {
       setlocale(LC_ALL, "");
       ros::init(argc, argv, "listener");
       ros::NodeHandle nh;
       ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter", 10, doMsg);
       ros::spin();
       return 0;
   }
   ```

3. **CMakeLists.txt Configuration**:
  
    Add flowing code in the end of your packages's `CMakeLists.txt`:
    ```cmake
    add_executable(listener src/listener.cpp)
    add_executable(talker src/talker.cpp)

    target_link_libraries(listener ${catkin_LIBRARIES})
    target_link_libraries(talker ${catkin_LIBRARIES})
    ```
    <p align="center">
      <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
      <img src="./images/CMakeLists.png" alt="J3010">
      </a>
    </p>

4. **Running the Code**:
  - Open a terminal and start `roscore`:
    ```bash
    roscore
    ```
  - In a new terminal, navigate to your workspace and run the publisher node:
    ```bash
    rosrun listener_and_talker listener
    ```
  - In another terminal, run the subscriber node:
    ```bash
    rosrun listener_and_talker talker
    ```
    <p align="center">
      <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
      <img src="./images/run_listener_and_talker.png" alt="J3010">
      </a>
    </p>

      <p align="center">
      <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
      <img src="./images/run_listener_and_talker_result.png" alt="J3010">
      </a>
    </p>
    
You should see messages being published and received, displayed in the terminal.

#### 3. Basic Topic Communication Operations (Python)

**Objective**: Create a publisher node that sends text messages at 10Hz and a subscriber node that prints the received messages.

**Steps**:

0. **[Create package](../6.1.2-Quick%20Experience%20with%20HelloWorld%20for%20ROS/README.md)**
    ```bash
    cd ~/seeed_ws/src/
    catkin_create_pkg listener_and_talker roscpp rospy std_msgs
    mkdir ~/seeed_ws/src/listener_and_talker/script
    cd ~/seeed_ws/src/listener_and_talker/script
    touch listener.py talker.py
    ```

1. **Publisher Implementation**:

    `talker.py`
   ```python
   #!/usr/bin/env python
   import rospy
   from std_msgs.msg import String

   if __name__ == "__main__":
       rospy.init_node("talker_p")
       pub = rospy.Publisher("chatter", String, queue_size=10)
       msg = String()
       msg_front = "hello 你好"
       count = 0
       rate = rospy.Rate(10)  # 10Hz

       while not rospy.is_shutdown():
           msg.data = msg_front + str(count)
           pub.publish(msg)
           rospy.loginfo("Sent message: %s", msg.data)
           rate.sleep()
           count += 1
   ```

2. **Subscriber Implementation**:

    `listener.py`
   ```python
   #!/usr/bin/env python
   import rospy
   from std_msgs.msg import String

   def doMsg(msg):
       rospy.loginfo("Heard: %s", msg.data)

   if __name__ == "__main__":
       rospy.init_node("listener_p")
       sub = rospy.Subscriber("chatter", String, doMsg, queue_size=10)
       rospy.spin()
   ```

3. **Add Executable Permissions**:
   ```bash
    sudo chmod +x *.py
   ```

4. **CMakeLists.txt Configuration**:

    Add flowing code in the end of your packages's `CMakeLists.txt`:
   ```cmake
   catkin_install_python(PROGRAMS
     scripts/talker.py
     scripts/listener.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

5. **Running the Code**:
   - Open a terminal and start `roscore`:
     ```bash
     roscore
     ```
   - In a new terminal, navigate to your workspace and run the publisher node:
     ```bash
     rosrun listener_and_talker talker.py
     ```
   - In another terminal, run the subscriber node:
     ```bash
     rosrun listener_and_talker listener.py
     ```

You should see messages being published and received, displayed in the terminal.

#### Conclusion

This tutorial introduced you to the basics of topic communication in ROS using both C++ and Python. By following the steps outlined above, you should be able to set up a simple publisher-subscriber model and understand how nodes communicate through topics. Feel free to explore further by modifying the messages and experimenting with different frequencies and message types.