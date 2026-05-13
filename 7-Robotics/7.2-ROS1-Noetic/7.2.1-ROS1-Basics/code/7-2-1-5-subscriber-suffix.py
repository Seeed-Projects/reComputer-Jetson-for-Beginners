#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose

def poseCallback(msg):
    rospy.loginfo("Turtle pose: x:%0.3f, y:%0.3f", msg.x, msg.y)

def turtle_pose_subscriber():

    rospy.init_node('turtle_pose_subscriber', anonymous=True)# Initialize the ROS node.

    # Create a subscriber for /turtle1/pose and register poseCallback.
    rospy.Subscriber("/turtle1/pose", Pose, poseCallback)


    rospy.spin()# Wait for callbacks.

if __name__ == '__main__':
    turtle_pose_subscriber()
