/**
This example calls the turtlesim /spawn service to create a new turtle at the specified position.
*/

#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "a_new_turtle");// Initialize the ROS node.

    ros::NodeHandle node;

    ros::service::waitForService("/spawn"); // Wait for the /spawn service.

    ros::ServiceClient new_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");//Create a service client for /spawn.

    // Initialize the turtlesim::Spawn request.
    turtlesim::Spawn new_turtle_srv;
    new_turtle_srv.request.x = 6.0;
    new_turtle_srv.request.y = 8.0;
    new_turtle_srv.request.name = "turtle2";

    // Call the service with x/y position and name parameters.
    ROS_INFO("Call service to create a new turtle name is %s,at the x:%.1f,y:%.1f", new_turtle_srv.request.name.c_str(),
        new_turtle_srv.request.x,
        new_turtle_srv.request.y);

    new_turtle.call(new_turtle_srv);


    ROS_INFO("Spawn turtle successfully [name:%s]", new_turtle_srv.response.name.c_str());// Display the service call result.

    return 0;
};
