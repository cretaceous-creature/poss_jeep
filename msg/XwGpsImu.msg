Header header

string msgId	##typically this data is for GPFPD

float32 systime

uint16 gpsWeek
float32 gpsTime

float32 heading  ##0~360  0 means toward north
float32 pitch    ##-90~90
float32 roll     ##-180~180

float32 lattitude
float32 longitude
float32 altitude

float32 ve ##east velocity
float32 vn ##north velocity
float32 vu ##vertical velocity

float32 baseline  ##baseline length, typically in jeep 1.707

uint16 nsv1  ##satellite num in antenna 1
uint16 nsv2  ##satellite num in antenna 2

uint16 status  ##typically 0x04, 0x05 and 0x0b with rtk station
uint16 parity  ##both saved as int
