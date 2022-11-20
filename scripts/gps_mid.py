#!/usr/bin/env python3

import serial
import types
import re
import rospy
import time
import threading

from datetime import datetime
from shipcon_pcc.msg import gps_position
from shipcon_pcc.msg import gps_local_position
from shipcon_pcc.msg import gps_navinfo
from shipcon_pcc.msg import gps_time


#Constants-----------------------------------------------------------------------------------------
PUBLISH_RATE = 10 #Hz
SERIAL_PORT = "/dev/ttyUSB2"
BAUD_RATE = 115200

#Class-----------------------------------------------------------------------------------------
class GpsSerial:
    def __init__(self, serial_port, baud_rate):
        self.gpsArr = []
        self.gpsStr = ""
        self.pos = gps_position()
        self.localpos = gps_local_position()
        self.navinfo = gps_navinfo()
        self.time = gps_time()
        self.ser = serial.Serial(
            port = serial_port,
            baudrate = baud_rate,
            parity = serial.PARITY_NONE,
            bytesize = serial.EIGHTBITS,
            stopbits = serial.STOPBITS_ONE,
            timeout = None,
            xonxoff = 0, #Enable software flow control
            rtscts = 0, #Enable Hardware(RTS,CTS) flow control
            )

        self.ser.write("$CMD,CHN,1,MSG,GGA,0.1\r\n".encode())
        self.ser.write("$CMD,CHN,1,MSG,GSV,1\r\n".encode())
        self.ser.write("$CMD,CHN,1,MSG,GSA,1\r\n".encode())
        self.ser.write("$CMD,CHN,1,MSG,ZDA,1\r\n".encode())
        self.ser.write("$CMD,CHN,1,MSG,VTG,1\r\n".encode()) #Speed and Course
        self.ser.write("$CMD,CHN,1,MSG,RMC,1\r\n".encode())
        self.ser.write("$CMD,CHN,1,MSG,SA3,1\r\n".encode())
        self.ser.write("$CMD,CHN,1,MSG,RME,1\r\n".encode())
        self.ser.write("$CMD,CHN,1,MSG,CLS,1\r\n".encode())
        self.ser.write("$CMD,CLAS,AUTO,ON\r\n".encode()) 
        
    def get_gps(self):
        self.gpsStr = self.ser.readline().decode()
        self.gpsArr = self.gpsStr.split(',')

        if re.match(r"^\$.*", self.gpsArr[0]):
            
            if self.gpsArr[0] == '$PTNL':
                if self.gpsArr[1] == 'GGK':
                    self.pos.latitude_deg = int(float(self.gpsArr[4]) // 100)
                    self.pos.latitude_min = float(self.gpsArr[4])  - (self.pos.latitude_deg * 100)
                    self.pos.latitude_dir = self.gpsArr[5]
                    self.pos.longitude_deg = int(float(self.gpsArr[6]) // 100)
                    self.pos.longitude_min = float(self.gpsArr[6])  - (self.pos.longitude_deg * 100)
                    self.pos.longitude_dir = self.gpsArr[7]
                    self.pos.gps_quality = int(self.gpsArr[8])
                    self.pos.satellite_number = int(self.gpsArr[9])
                elif self.gpsArr[1] == 'VGK':
                    if self.gpsArr[4] =='-' or self.gpsArr[4] == '':
                        pass
                    else:
                        self.localpos.localpos_e = float(self.gpsArr[4])#float( re.sub(r'\+', "", self.gpsArr[4]) )
                    if self.gpsArr[5] =='-' or self.gpsArr[5] == '':
                        pass
                    else:
                        self.localpos.localpos_n = float(self.gpsArr[5])#float( re.sub(r'\+', "", self.gpsArr[5]) )
                    if self.gpsArr[6] =='-' or self.gpsArr[6] == '':
                        pass
                    else:
                        self.localpos.localpos_z = float(self.gpsArr[6])#float( re.sub(r'\+', "", self.gpsArr[6]) )
                elif self.gpsArr[1] == 'VHD':
                    self.localpos.local_azimath = float(self.gpsArr[4])
                    self.localpos.local_vertical = float(self.gpsArr[6])
                    self.localpos.local_range = float(self.gpsArr[8])
                
            elif self.gpsArr[0] == '$GPGST':
                self.pos.latitude_err = float(self.gpsArr[6])
                self.pos.longitude_err = float(self.gpsArr[7])
                self.pos.ellipse_err_maj = float(self.gpsArr[3])
                self.pos.ellipse_err_min = float(self.gpsArr[4])
                self.navinfo.truenorth_heading_err = float(self.gpsArr[5])
        
            elif self.gpsArr[0] == '$GPROT':
                if self.gpsArr[1] =='-' or self.gpsArr[1] == '':
                    pass
                else:
                    self.navinfo.rotation_rate = float(self.gpsArr[1])
                    
            elif self.gpsArr[0] == '$GPVTG':
                if self.gpsArr[1] =='' or self.gpsArr[5]=='' or self.gpsArr[7]=='':
                    pass
                else:
                    self.navinfo.truenorth_heading = float(self.gpsArr[1])
                    self.navinfo.speed_knot = float(self.gpsArr[5])
                    self.navinfo.speed_km = float(self.gpsArr[7])
            
            elif self.gpsArr[0] == '$GPZDA':
                tmptime = datetime.strptime(self.gpsArr[1]+','+self.gpsArr[4]+self.gpsArr[3]+self.gpsArr[2], '%H%M%S.%f,%Y%m%d')
                self.time.day = tmptime.day
                self.time.month = tmptime.month
                self.time.year = tmptime.year
                self.time.hour = tmptime.hour
                self.time.min = tmptime.minute
                self.time.sec = tmptime.second

            elif self.gpsArr[0] == '$GPGGA':
                self.pos.latitude_deg = int(float(self.gpsArr[2]) // 100)
                self.pos.latitude_min = float(self.gpsArr[2])  - (self.pos.latitude_deg * 100)
                self.pos.latitude_dir = self.gpsArr[3]
                self.pos.longitude_deg = int(float(self.gpsArr[4]) // 100)
                self.pos.longitude_min = float(self.gpsArr[4])  - (self.pos.longitude_deg * 100)
                self.pos.longitude_dir = self.gpsArr[5]
                self.pos.gps_quality = int(self.gpsArr[6])
                self.pos.satellite_number = int(self.gpsArr[7]) 

class RosPublish:
    def __init__(self):
        self.pub_pos = rospy.Publisher('gps_position2', gps_position, queue_size = 1)
        self.pub_localpos = rospy.Publisher('gps_localposition2', gps_local_position, queue_size = 1)
        self.pub_navinfo = rospy.Publisher('gps_navinfo2', gps_navinfo, queue_size = 1)
        self.pub_time = rospy.Publisher('gps_time2', gps_time, queue_size = 1)

    def ros_log(self, pos, localpos, navinfo, time):
        rospy.loginfo(pos)
        rospy.loginfo(localpos)
        rospy.loginfo(navinfo)
        rospy.loginfo(time)

    def ros_pub(self, pos, localpos, navinfo, time):
        self.pub_pos.publish(pos)
        self.pub_localpos.publish(localpos)
        self.pub_navinfo.publish(navinfo)
        self.pub_time.publish(time)        



#Function-----------------------------------------------------------------------------------------
def publish_gps(gps):
    rospy.init_node('NODE_Gps2', anonymous = True)
    ros = rospy.Rate(PUBLISH_RATE)
    pub = RosPublish()

    pub.ros_log(gps.pos, gps.localpos, gps.navinfo, gps.time)
    pub.ros_pub(gps.pos, gps.localpos, gps.navinfo, gps.time)
    ros.sleep()


def refresh_gps(gps):
    while not rospy.is_shutdown():
        gps.get_gps()
        


#Main-----------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        gps = GpsSerial(SERIAL_PORT, BAUD_RATE)
        thread_1 = threading.Thread(target = refresh_gps, args = (gps,))
        thread_1.start()

        while not rospy.is_shutdown():
            publish_gps(gps)

    except rospy.ROSInterruptException:
        pass
