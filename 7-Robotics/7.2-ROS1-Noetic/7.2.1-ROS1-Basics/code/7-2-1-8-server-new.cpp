/**
This example provides /turtle_vel_command with the std_srvs/Trigger service type.
*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

ros::Publisher turtle_vel_pub;
bool pubvel = false;

// Service callback: req is the request and res is the response.
bool pubvelCallback(std_srvs::Trigger::Request  &req,
                    std_srvs::Trigger::Response &res)
{
    pubvel = !pubvel;

        ROS_INFO("Do you want to publish the vel?: [%s]", pubvel==true?"Yes":"No");// Print the client request.

    // Set response data.
    res.success = true;
    res.message = "The status is changed!";

    return true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "turtle_vel_command_server");


    ros::NodeHandle n;

    // Create the /turtle_vel_command server and register pubvelCallback.
    ros::ServiceServer command_service = n.advertiseService("/turtle_vel_command", pubvelCallback);

    // Create a publisher for /turtle1/cmd_vel. The message type is geometry_msgs::Twist and the queue size is 8.
    turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 8);

    ros::Rate loop_rate(10);// Set the loop rate.

    while(ros::ok())
    {

        ros::spinOnce();// Process callbacks once.

        // Publish turtle velocity commands when pubvel is true.
        if(pubvel)
        {
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0.6;
            vel_msg.angular.z = 0.8;
            turtle_vel_pub.publish(vel_msg);
        }

        loop_rate.sleep();//Sleep according to the loop rate.
    }

    return 0;
}
