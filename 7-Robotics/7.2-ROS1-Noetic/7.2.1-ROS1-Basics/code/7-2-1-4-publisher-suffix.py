#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def turtle_velocity_publisher():

    rospy.init_node('turtle_velocity_publisher', anonymous=True) # Initialize the ROS node.

    # Create a turtlesim velocity publisher on /turtle1/cmd_vel. The message type is geometry_msgs/Twist and the queue size is 8.
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=8)


    rate = rospy.Rate(10) # Set the loop rate.

    while not rospy.is_shutdown():
        # Initialize a geometry_msgs::Twist message.
        turtle_vel_msg = Twist()
        turtle_vel_msg.linear.x = 0.8
        turtle_vel_msg.angular.z = 0.6

        # Publish the message.
        turtle_vel_pub.publish(turtle_vel_msg)
        rospy.loginfo("linear is:%0.2f m/s, angular is:%0.2f rad/s",
                turtle_vel_msg.linear.x, turtle_vel_msg.angular.z)


        rate.sleep()# Sleep according to the loop rate.

if __name__ == '__main__':
    try:
        turtle_velocity_publisher()
    except rospy.ROSInterruptException:
        pass
