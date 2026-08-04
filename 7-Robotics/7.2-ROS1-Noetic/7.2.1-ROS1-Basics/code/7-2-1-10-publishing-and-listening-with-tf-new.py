#!/usr/bin/env python3

import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()# Create a TF broadcaster.
    # Broadcast the TF transform between world and the named turtle.
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':

    rospy.init_node('turtle1_turtle2_tf_broadcaster')# Initialize the ROS node.

    turtlename = rospy.get_param('~turtle') # Get the turtle name from the parameter server.
    # Subscribe to the turtle pose topic.
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
