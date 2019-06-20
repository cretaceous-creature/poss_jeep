#ifndef JEEPCANBUS_H
#define JEEPCANBUS_H

#define OPENSUCCESS 1

#include <unistd.h>
#include <ros/ros.h>
#include "controlcan_unix.h"
#include "poss_jeep/CanRaw.h"

class JeepCanbus {
public:
    JeepCanbus(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~JeepCanbus();
    void processData();

    VCI_INIT_CONFIG InitInfo;
    VCI_CAN_OBJ receivedata[2500];
    int m_DevIndex;
    int m_DevType;
    unsigned long Reserved;

public:
    bool open();
    bool close();

private:
    bool isRunning;
    ros::NodeHandle nh;
    ros::NodeHandle nhp;
    ros::Publisher rawdata_pub;

    int count;
};

#endif // JEEPCANBUS_H
