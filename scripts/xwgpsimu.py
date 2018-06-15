#!/usr/bin/env python

import rospy
import roslib
import rospkg
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
        self.updaterate = 100    #15hz
        #initialize the parameters
        self.port = rospy.get_param("~port", self.port)
        self.baud = rospy.get_param('~baud', self.baud)
        self.verbose = rospy.get_param('~verbose', self.verbose)
        self.verbose = rospy.get_param('~savelog', self.savelog)
        self.serialtimeout = rospy.get_param('~serialtimeout', self.serialtimeout)
        self.updaterate = rospy.get_param('~updaterate', self.updaterate)
        rospy.loginfo("port is : %s" % self.port)
        rospy.loginfo("baudrate is: %s " % self.baud )
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
            t = time.time()
            rospack = rospkg.RosPack()
            self.logfile = open(rospack.get_path("poss_jeep")+"/log/" +"gpslog" + str(t) + '.log', 'w')


    #delete function
    def __del__(self):
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
                    self.logfile.write(data_in)
                    
                index = data_in.find(self.header)
                if index == 0:
                    datasplitdone = data_in.split(",") ##comma to split
                    self.XwGps.header.stamp = rospy.Time.now()
                    self.XwGps.msgId = datasplitdone[0]
                    self.XwGps.gpsWeek = int(datasplitdone[1])
                    self.XwGps.gpsTime = float(datasplitdone[2])
                    self.XwGps.heading = float(datasplitdone[3])
                    self.XwGps.pitch = float(datasplitdone[4])
                    self.XwGps.roll = float(datasplitdone[5])
                    self.XwGps.lattitude = float(datasplitdone[6])
                    self.XwGps.longitude = float(datasplitdone[7])
                    self.XwGps.altitude = float(datasplitdone[8])
                    self.XwGps.ve = float(datasplitdone[9])
                    self.XwGps.vn = float(datasplitdone[10])
                    self.XwGps.vu = float(datasplitdone[11])
                    self.XwGps.baseline = float(datasplitdone[12])
                    self.XwGps.nsv1 = int(datasplitdone[13])
                    self.XwGps.nsv2 = int(datasplitdone[14])
                    finaltwo = datasplitdone[15].split('*') # status*parity
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

