#include <ros/ros.h>
#include "learning_server/IntPlus.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IntPlus_client");
    ros::NodeHandle nh;
    ros::service::waitForService("/Two_Int_Plus");
    ros::ServiceClient client = nh.serviceClient<learning_server::IntPlus>("/Two_Int_Plus");

    learning_server::IntPlus srv;
    srv.request.a = 8;
    srv.request.b = 6;

    if (client.call(srv)) {
        ROS_INFO("Result: %ld", srv.response.result);
    } else {
        ROS_ERROR("Failed to call /Two_Int_Plus");
    }
    return 0;
}
