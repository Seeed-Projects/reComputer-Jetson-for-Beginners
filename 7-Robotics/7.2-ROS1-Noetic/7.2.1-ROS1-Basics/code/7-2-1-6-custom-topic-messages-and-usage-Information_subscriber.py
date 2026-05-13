#!/usr/bin/env python3
import rospy
from learning_topic.msg import Information

def company_info_callback(msg):
    rospy.loginfo('Company: %s, city: %s', msg.company, msg.city)

def information_subscriber():
    rospy.init_node('information_subscriber', anonymous=True)
    rospy.Subscriber('/company_info', Information, company_info_callback)
    rospy.spin()

if __name__ == '__main__':
    information_subscriber()
