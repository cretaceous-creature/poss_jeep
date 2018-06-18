#include "jeep_canbus.h"

JeepCanbus::JeepCanbus(ros::NodeHandle nh, ros::NodeHandle nhp)
    : nh(nh), nhp(nhp) {
    isRunning = false;
    rawdata_pub = nh.advertise<poss_jeep::CanRaw>("/can_raw", 5);
}

JeepCanbus::~JeepCanbus() {
    if (isRunning) {
        close();
    }
}

void JeepCanbus::processData() {
    int len = VCI_Receive(m_DevType, m_DevIndex, 0, receivedata, 2500, 200);
    for (int i = 0; i < len; i++) {
        if (receivedata[i].RemoteFlag != 0 || receivedata[i].ExternFlag != 0) {
            continue;
        }
        unsigned int id = receivedata[i].ID;
        if (id != 0x1E1 && id != 0x211 && id != 0x20E) continue;
        poss_jeep::CanRaw canbus_msg;
        canbus_msg.header.stamp = ros::Time();
        canbus_msg.header.frame_id = "canbus";
        canbus_msg.id = id;
        int datalen = receivedata[i].DataLen;
        for (int k = 0; k < datalen; k++) {
            canbus_msg.data[k] =  receivedata[i].Data[k];
        }
        rawdata_pub.publish(canbus_msg);
    }
}

bool JeepCanbus::open() {
    if (isRunning) return false;

    m_DevIndex = 0;
    m_DevType = VCI_USBCAN2;
    Reserved = 0;
    InitInfo.Timing0 = 0x03; // 03 for jeep. 00 for c4
    InitInfo.Timing1 = 0x1C;
    InitInfo.Filter = 0;
    InitInfo.AccCode = 0x80000000;
    InitInfo.AccMask = 0xFFFFFFFF;
    InitInfo.Mode = 1;

    if (VCI_OpenDevice(m_DevType, m_DevIndex, Reserved) != OPENSUCCESS) {
        return false;
    }

    ROS_INFO( "\033[32mCanbus device is open!\033[0m");
    usleep(100000);
    if(VCI_InitCAN(m_DevType, m_DevIndex, 0, &InitInfo) != OPENSUCCESS) {
        return false;
    }
    ROS_INFO( "\033[32mCanbus device is initialized!\033[0m");
    usleep(100000);
    if (VCI_StartCAN(m_DevType, m_DevIndex, 0) != OPENSUCCESS) {
        return false;
    }
    ROS_INFO( "\033[32mCanbus device is started!\033[0m");
    return true;
}

bool JeepCanbus::close() {
    if (!isRunning) return false;
    isRunning = false;
    if (VCI_CloseDevice(m_DevType, m_DevIndex) != OPENSUCCESS) {
        return false;
    }
    return true;
}
