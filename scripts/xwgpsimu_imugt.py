#!/usr/bin/env python

import rospy
import roslib
import rospkg
import os
import serial
import time

roslib.load_manifest('poss_jeep')
from poss_jeep.msg import XwGpsImu
from std_msgs.msg import Header

class Serial_XWGPS:
    def __init__(self):
        self.header = '$GPFPD'
        self.port = '/dev/ttyS1'
        self.baud = 115200
        self.verbose = True
        self.savelog = True
        self.serialtimeout = 0.1
        self.updaterate = 200    #100hz
        self.gpsrate = 0.01
        #initialize the parameters
        self.port = rospy.get_param("~port", self.port)
        self.baud = rospy.get_param('~baud', self.baud)
        self.verbose = rospy.get_param('~verbose', self.verbose)
        self.savelog = rospy.get_param('~savelog', self.savelog)
        self.serialtimeout = rospy.get_param('~serialtimeout', self.serialtimeout)
        self.updaterate = rospy.get_param('~updaterate', self.updaterate)
        self.gpsrate = rospy.get_param('~gpsrate', self.gpsrate) 
        self.count = 0
        rospy.loginfo("port is : %s" % self.port)
        rospy.loginfo("baudrate is: %s " % self.baud) 
        rospy.loginfo("gpsrate is: %s " % self.gpsrate)
        rospy.loginfo("updaterate is: %s " % self.updaterate) 
        self.ser = serial.Serial(self.port, self.baud, timeout=self.serialtimeout)
        if not (self.ser.isOpen()):
            rospy.ROSException("Cant open port %s" % self.baud)
            
        #gps imu publisher

        #write a log file
        if self.savelog:             
            t = time.strftime("%Y-%m-%d-%H-%M-%S",time.localtime())
            rospack = rospkg.RosPack()
            self.logfile = open(os.environ['HOME'] +"/data/imulog" + t + '.log', 'w')

        #start to send data
        #self.ser.write("$cmd,set,com1,115200,none,8,1,rs232,log*ff\r\n")
        #self.ser.write("$cmd,get,com*ff\r\n")
        self.ser.write("$cmd,output,com1,gtimu,%.2f*ff\r\n" % self.gpsrate)

    
    #delete function
    def __del__(self):
        self.ser.write("$cmd,output,com1,null*FF\r\n")
        self.ser.close()
        self.logfile.close()

    def Serial_Update(self):
        #always read the data
        r = rospy.Rate(self.updaterate)
        while not rospy.is_shutdown():

            data_in = self.ser.readline()
            if data_in:
                if self.verbose: ## for debug the port
                    if self.count <= 0:
                        rospy.loginfo(data_in.strip())
                        #data will be like $GPFPD, , , , ,blabla
                 
                    self.count += 1
                    if self.count >= 100:
                        self.count = 0
                if self.savelog:
                    self.logfile.write(('%.6f,' % time.time()) + data_in)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('XwIMU')
    ex = Serial_XWGPS()
    try:
        ex.Serial_Update()
    except rospy.ROSInterruptException: pass


