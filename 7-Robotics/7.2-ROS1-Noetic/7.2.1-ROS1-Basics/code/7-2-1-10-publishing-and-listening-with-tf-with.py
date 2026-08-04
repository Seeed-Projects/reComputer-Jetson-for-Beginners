#!/usr/bin/env python3
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')# Initialize the ROS node.

    listener = tf.TransformListener()# Initialize a TF listener.

    rospy.wait_for_service('spawn')
    # Call the service to create another turtle named turtle2.
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(8, 6, 0, 'turtle2')
    # Declare a publisher for turtle2 velocity.
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Look up the TF transform between turtle2 and turtle1.
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # Compute linear and angular velocity, then publish them.
        angular = 6.0 * math.atan2(trans[1], trans[0])
        linear = 0.8 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)
        rate.sleep()
