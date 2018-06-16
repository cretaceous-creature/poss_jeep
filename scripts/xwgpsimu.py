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
        self.serialtimeout = 0.01
        self.updaterate = 100    #100hz
        self.gpsrate = 0.01
        #initialize the parameters
        self.port = rospy.get_param("~port", self.port)
        self.baud = rospy.get_param('~baud', self.baud)
        self.verbose = rospy.get_param('~verbose', self.verbose)
        self.verbose = rospy.get_param('~savelog', self.savelog)
        self.serialtimeout = rospy.get_param('~serialtimeout', self.serialtimeout)
        self.updaterate = rospy.get_param('~updaterate', self.updaterate)
        self.gpsrate = rospy.get_param('~gpsrate', self.gpsrate) 
        rospy.loginfo("port is : %s" % self.port)
        rospy.loginfo("baudrate is: %s " % self.baud) 
        rospy.loginfo("gpsrate is: %s " % self.gpsrate)
        rospy.loginfo("updaterate is: %s " % self.updaterate) 
        self.ser = serial.Serial(self.port, self.baud, timeout=self.serialtimeout)
        if not (self.ser.isOpen()):
            rospy.ROSException("Cant open port %s" % self.baud)
            
        #gps imu publisher

        self.XwGps = XwGpsImu()
        #XwGps.header = Header
        self.XwGps.header.frame_id = "/gpsimu"
        self._pub_XwGps = rospy.Publisher("/GPSIMUdata", XwGpsImu, queue_size = 1)
        #write a log file
        if self.savelog:             
            t = time.strftime("%Y-%m-%d-%H-%M-%S",time.localtime())
            rospack = rospkg.RosPack()
            #self.logfile = open(rospack.get_path("poss_jeep")+"/log/" +"gpslog" + t + '.log', 'w')
            self.logfile = open(os.environ['HOME'] +"/data/gpslog" + t + '.log', 'w')

        #start to send data
        self.ser.write("$cmd,output,com0,gpfpd,%.2f*ff\r\n" % self.gpsrate)


    
    #delete function
    def __del__(self):
        self.ser.write("$cmd,output,com0,null*FF\r\n")
        self.ser.close()
        self.logfile.close()

    def Serial_Update(self):
        #always read the data
        r = rospy.Rate(self.updaterate)
        while not rospy.is_shutdown():

            data_in = self.ser.readline()
            if data_in:
                if self.verbose: ## for debug the port
                    rospy.loginfo(data_in)
                    #data will be like $GPFPD, , , , ,blabla
                if self.savelog:
                    self.logfile.write(str(time.time()) + data_in)
                    
                index = data_in.find(self.header)
                if index == 0:
                    datasplitdone = data_in.split(",") ##comma to split
                    self.XwGps.header.stamp = rospy.Time.now()
                    self.XwGps.msgId = datasplitdone[0] if datasplitdone[0] else ''
                    self.XwGps.systime = time.time()
                    rospy.loginfo(self.XwGps.systime)
                    self.XwGps.gpsWeek = int(datasplitdone[1]) if datasplitdone[1] else -1
                    self.XwGps.gpsTime = float(datasplitdone[2]) if datasplitdone[2] else -1
                    self.XwGps.heading = float(datasplitdone[3]) if datasplitdone[3] else -1
                    self.XwGps.pitch = float(datasplitdone[4])  if datasplitdone[4] else -1
                    self.XwGps.roll = float(datasplitdone[5]) if datasplitdone[5] else -1
                    self.XwGps.lattitude = float(datasplitdone[6]) if datasplitdone[6] else -1
                    self.XwGps.longitude = float(datasplitdone[7]) if datasplitdone[7] else -1
                    self.XwGps.altitude = float(datasplitdone[8]) if datasplitdone[8] else -1
                    self.XwGps.ve = float(datasplitdone[9]) if datasplitdone[9] else -1
                    self.XwGps.vn = float(datasplitdone[10]) if datasplitdone[10] else -1
                    self.XwGps.vu = float(datasplitdone[11]) if datasplitdone[11] else -1
                    self.XwGps.baseline = float(datasplitdone[12]) if datasplitdone[12] else -1
                    self.XwGps.nsv1 = int(datasplitdone[13]) if datasplitdone[13] else -1
                    self.XwGps.nsv2 = int(datasplitdone[14]) if datasplitdone[14] else -1
                    # status*parity
                    finaltwo = datasplitdone[15].split('*')
                    self.XwGps.status = int(finaltwo[0],16)
                    self.XwGps.parity = int(finaltwo[1],16)
                    self._pub_XwGps.publish(self.XwGps)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('XwGPS')
    ex = Serial_XWGPS()
    try:
        ex.Serial_Update()
    except rospy.ROSInterruptException: pass

