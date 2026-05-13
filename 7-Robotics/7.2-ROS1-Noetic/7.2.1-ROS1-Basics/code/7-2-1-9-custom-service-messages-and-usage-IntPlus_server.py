#!/usr/bin/env python3
import rospy
from learning_server.srv import IntPlus, IntPlusResponse

def int_plus_callback(req):
    rospy.loginfo('Ints: a:%d b:%d', req.a, req.b)
    return IntPlusResponse(req.a + req.b)

if __name__ == '__main__':
    rospy.init_node('IntPlus_server')
    rospy.Service('/Two_Int_Plus', IntPlus, int_plus_callback)
    rospy.loginfo('Ready to calculate two integers.')
    rospy.spin()
