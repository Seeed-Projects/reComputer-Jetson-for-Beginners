/**
 * Publish /company_info with the custom learning_topic::Information message type.
 */
#include <ros/ros.h>
#include "learning_topic/Information.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "company_information_publisher");
    ros::NodeHandle nh;

    ros::Publisher info_pub = nh.advertise<learning_topic::Information>("/company_info", 10);
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        learning_topic::Information info_msg;
        info_msg.company = "Seeed";
        info_msg.city = "Shenzhen";

        info_pub.publish(info_msg);
        ROS_INFO("Information: company:%s city:%s", info_msg.company.c_str(), info_msg.city.c_str());
        loop_rate.sleep();
    }
    return 0;
}
