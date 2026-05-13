/*Create a subscriber for the current turtlesim pose.*/
#include <ros/ros.h>
#include "turtlesim/Pose.h"
// The callback runs when a subscribed message is received.
void turtle_poseCallback(const turtlesim::Pose::ConstPtr& msg){
    // Print the received message.
    ROS_INFO("Turtle pose: x:%0.3f, y:%0.3f", msg->x, msg->y);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "turtle_pose_subscriber");// Initialize the ROS node.

    ros::NodeHandle n;//Create a node handle.

    // Create a subscriber for /turtle1/pose and register poseCallback.
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, turtle_poseCallback);

    ros::spin(); // Wait for callbacks.

    return 0;
}
