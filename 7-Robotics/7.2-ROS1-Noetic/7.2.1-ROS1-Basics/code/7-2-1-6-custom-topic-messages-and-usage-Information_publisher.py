#!/usr/bin/env python3
import rospy
from learning_topic.msg import Information

def information_publisher():
    rospy.init_node('information_publisher', anonymous=True)
    info_pub = rospy.Publisher('/company_info', Information, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        info_msg = Information()
        info_msg.company = 'Seeed'
        info_msg.city = 'Shenzhen'
        info_pub.publish(info_msg)
        rospy.loginfo('Information: company:%s city:%s', info_msg.company, info_msg.city)
        rate.sleep()

if __name__ == '__main__':
    information_publisher()
