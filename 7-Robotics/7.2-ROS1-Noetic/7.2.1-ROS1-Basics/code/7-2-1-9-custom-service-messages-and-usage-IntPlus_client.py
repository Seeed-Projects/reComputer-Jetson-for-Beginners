#!/usr/bin/env python3
import rospy
from learning_server.srv import IntPlus

if __name__ == '__main__':
    rospy.init_node('IntPlus_client')
    rospy.wait_for_service('/Two_Int_Plus')
    plus_client = rospy.ServiceProxy('/Two_Int_Plus', IntPlus)
    response = plus_client(22, 20)
    rospy.loginfo('Result: %d', response.result)
