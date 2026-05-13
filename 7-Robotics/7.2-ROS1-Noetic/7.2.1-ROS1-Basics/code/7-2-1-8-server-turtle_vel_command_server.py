#!/usr/bin/env python3

import threading

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

pubvel = False
turtle_vel_pub = None


def publish_velocity_loop():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if pubvel:
            vel_msg = Twist()
            vel_msg.linear.x = 0.6
            vel_msg.angular.z = 0.8
            turtle_vel_pub.publish(vel_msg)
        rate.sleep()


def pubvel_callback(req):
    global pubvel
    pubvel = not pubvel
    rospy.loginfo('Publish turtle velocity: %s', pubvel)
    return TriggerResponse(success=True, message='Velocity publishing toggled.')


def turtle_pubvel_command_server():
    global turtle_vel_pub
    rospy.init_node('turtle_vel_command_server')
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=8)
    rospy.Service('/turtle_vel_command', Trigger, pubvel_callback)
    threading.Thread(target=publish_velocity_loop, daemon=True).start()
    rospy.loginfo('Ready to receive /turtle_vel_command requests.')
    rospy.spin()


if __name__ == '__main__':
    turtle_pubvel_command_server()
