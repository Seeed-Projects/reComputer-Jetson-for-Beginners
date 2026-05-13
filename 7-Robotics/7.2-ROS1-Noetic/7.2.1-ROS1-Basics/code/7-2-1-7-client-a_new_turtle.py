#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn


def turtle_spawn():
    rospy.init_node('new_turtle')
    rospy.wait_for_service('/spawn')

    try:
        spawn_client = rospy.ServiceProxy('/spawn', Spawn)
        response = spawn_client(2.0, 2.0, 0.0, 'turtle2')
        return response.name
    except rospy.ServiceException as exc:
        rospy.logerr('Failed to call /spawn: %s', exc)
        return None


if __name__ == '__main__':
    name = turtle_spawn()
    if name:
        rospy.loginfo('Created a new turtle named %s.', name)
