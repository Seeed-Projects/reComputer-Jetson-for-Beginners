#include <ros/ros.h>
#include "learning_server/IntPlus.h"

bool intPlusCallback(learning_server::IntPlus::Request &req,
                     learning_server::IntPlus::Response &res)
{
    ROS_INFO("number 1 is:%ld, number 2 is:%ld", req.a, req.b);
    res.result = req.a + req.b;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IntPlus_server");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("/Two_Int_Plus", intPlusCallback);
    ROS_INFO("Ready to calculate two integers.");
    ros::spin();
    return 0;
}
