/*Create a turtlesim velocity publisher.*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char **argv){

    ros::init(argc, argv, "turtle_velocity_publisher");//Initialize the ROS node.

    ros::NodeHandle n;//Create a node handle.

    //Create a publisher for /turtle1/cmd_vel with geometry_msgs::Twist messages and queue size 10.
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::Rate loop_rate(10);//Set the loop rate.

    while (ros::ok()){
            //Initialize a message with the same type as the publisher.
        geometry_msgs::Twist turtle_vel_msg;
        turtle_vel_msg.linear.x = 0.8;
        turtle_vel_msg.angular.z = 0.6;

        turtle_vel_pub.publish(turtle_vel_msg);// Publish the velocity message.

        //Print the published velocity.
        ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", turtle_vel_msg.linear.x, turtle_vel_msg.angular.z);

        loop_rate.sleep();//Sleep according to the loop rate.
    }
    return 0;
}
