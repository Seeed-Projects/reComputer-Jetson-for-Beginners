/**
 * Subscribe to /company_info with the custom learning_topic::Information message type.
 */
#include <ros/ros.h>
#include "learning_topic/Information.h"

void companyInfoCallback(const learning_topic::Information::ConstPtr& msg)
{
    ROS_INFO("Company: %s, city: %s", msg->company.c_str(), msg->city.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "company_information_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/company_info", 10, companyInfoCallback);
    ros::spin();
    return 0;
}
