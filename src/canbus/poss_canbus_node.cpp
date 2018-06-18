#include <ros/ros.h>
#include "jeep_canbus.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "poss_canbus");
    ros::NodeHandle nh("jeep_canbus");
    ros::NodeHandle nhp("~");

    JeepCanbus *canbus = new JeepCanbus(nh, nhp);
    if (!canbus->open()) { return -1; }

    ros::Rate rate(1000);
    while (ros::ok()) {
        ros::spinOnce();
        canbus->processData();
        rate.sleep();
    }


    delete canbus;

    return 0;
}
