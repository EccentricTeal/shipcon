#!/usr/bin/env python

import types
import re
import rospy
import time
import threading
import os

from datetime import datetime
from shipcon.msg import gps_position, gps_local_position, gps_navinfo, gps_time, gyro, motor_info
from std_msgs.msg import Int16, Float32


#Constants-------------------------------------------------------------------------------------

#Class-----------------------------------------------------------------------------------------
class LoggerClass:
    LOG_PATH = "/home/ubuntu/catkin_ws/log/"
    FILELOG_RATE = 4 #Hz

    ##Constructor
    def __init__(self):
        #Instance Variables
        #Flag
        self.__autoflag = 0
        
        #GPS
        self.__gpspos = gps_position()
        self.__gpsloc = gps_local_position()
        self.__gpsnav = gps_navinfo()
        self.__gpstime = gps_time()

        #Gyro
        self.__gyro = gyro()

        #Actuators
        self.__motor1_info = motor_info()
        self.__rudder1_info = Float32()

        #Controller
        self.__autosw = Int16()
        self.__rcmotor1 = Int16()
        self.__rcrudder1 = Int16()

        #AutoSignal
        self.__automotor1 = Int16()
        self.__autorudder1 = Int16()

        self.locker = threading.Lock()

        #Initialize ROS
        rospy.init_node('NODE_Logger', anonymous = True)
        self.__sub_gpspos = rospy.Subscriber("gps_position", gps_position, self.__gpspos_cb)
        self.__sub_gpsloc = rospy.Subscriber("gps_localposition", gps_local_position, self.__gpsloc_cb)
        self.__sub_gpsnav = rospy.Subscriber("gps_navinfo", gps_navinfo, self.__gpsnav_cb)
        self.__sub_gpstime = rospy.Subscriber("gps_time", gps_time, self.__gpstime_cb)
        self.__sub_gyro = rospy.Subscriber("gyro", gyro, self.__gyro_cb)
        self.__sub_motor1_info = rospy.Subscriber("motor1_info", motor_info, self.__motor1_info_cb)
        self.__sub_rudder1_info = rospy.Subscriber("rudder1_info", Float32, self.__rudder1_info_cb)
        self.__sub_autosw = rospy.Subscriber("rc_autosw", Int16, self.__autosw_cb)
        self.__sub_rcmotor1 = rospy.Subscriber("rc_motor1", Int16, self.__rcmotor1_cb)
        self.__sub_rcrudder1 = rospy.Subscriber("rc_rudder1", Int16, self.__rcrudder1_cb)
        self.__sub_automotor1 = rospy.Subscriber("auto_motor1", Int16, self.__automotor1_cb)
        self.__sub_autorudder1 = rospy.Subscriber("auto_rudder1", Int16, self.__autorudder1_cb)

        #Initialize File
        date = datetime.now()
        self.__dirname = self.LOG_PATH + date.strftime('%Y%m%d') + '/'
        try:
            os.mkdir(self.__dirname)
        except:
            pass
        self.__filename = self.__dirname + date.strftime('%H%M%S_')

        #Initialize Thread
        self.th_print = threading.Thread(target = self.printout)
        self.th_filelog = threading.Thread(target = self.filelog)
        self.th_auto_watchflag = threading.Thread(target = self.auto_watchflag)
        self.th_print.start()
        self.th_filelog.start()
        self.th_auto_watchflag.start()

        #SPINNING
        rospy.spin()

        
    def __gpspos_cb(self, msg):
        self.locker.acquire()
        self.__gpspos = msg
        self.locker.release()

    def __gpsloc_cb(self, msg):
        self.locker.acquire()
        self.__gpsloc = msg
        self.locker.release()

    def __gpsnav_cb(self, msg):
        self.locker.acquire()
        self.__gpsnav = msg
        self.locker.release()

    def __gpstime_cb(self, msg):
        self.locker.acquire()
        self.__gpstime = msg
        self.locker.release()

    def __gyro_cb(self, msg):
        self.locker.acquire()
        self.__gyro = msg
        self.locker.release()

    def __motor1_info_cb(self, msg):
        self.locker.acquire()
        self.__motor1_info = msg
        self.locker.release()

    def __rudder1_info_cb(self, msg):
        self.locker.acquire()
        self.__rudder1_info = msg
        self.locker.release()

    def __autosw_cb(self,msg):
        self.locker.acquire()
        self.__autosw = msg
        self.locker.release()

    def __rcmotor1_cb(self,msg):
        self.locker.acquire()
        self.__rcmotor1 = msg
        self.locker.release()

    def __rcrudder1_cb(self,msg):
        self.locker.acquire()
        self.__rcrudder1 = msg
        self.locker.release()

    def __automotor1_cb(self,msg):
        self.locker.acquire()
        self.__automotor1 = msg
        self.locker.release()

    def __autorudder1_cb(self,msg):
        self.locker.acquire()
        self.__autorudder1 = msg
        self.locker.release()
        
        
    def printout(self):
        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            self.locker.acquire()
            rospy.loginfo("--GPSTime JST--  [%d/%02d/%02d  %02d:%02d:%02d]", self.__gpstime.year, self.__gpstime.month, self.__gpstime.day, self.__gpstime.hour + 9, self.__gpstime.min, self.__gpstime.sec)
            
            print("---------------------------Sensor---------------------------")
            rospy.loginfo("<GPS [SAT:%d]>  %s%d,%f / %s%d,%f", self.__gpspos.satellite_number, self.__gpspos.latitude_dir, self.__gpspos.latitude_deg, self.__gpspos.latitude_min, self.__gpspos.longitude_dir, self.__gpspos.longitude_deg, self.__gpspos.longitude_min)
            
            rospy.loginfo("<GPS-LOC>      (X,Y,Z) = (%02.3f, %02.3f, %02.3f)  / (Phi, Theta, R) = (%02.3f, %02.3f, %02.3f)", self.__gpsloc.localpos_e, self.__gpsloc.localpos_n, self.__gpsloc.localpos_z, self.__gpsloc.local_azimath, self.__gpsloc.local_vertical, self.__gpsloc.local_range)

            rospy.loginfo("<Gyro-Ang>     (Roll,Pitch,Yaw) = (%02.3f, %02.3f, %02.3f)", self.__gyro.ang_roll, self.__gyro.ang_pitch, self.__gyro.ang_yaw)            
            rospy.loginfo("<Gyro-Vel>     (Roll,Pitch,Yaw) = (%02.3f, %02.3f, %02.3f)", self.__gyro.vel_roll, self.__gyro.vel_pitch, self.__gyro.vel_yaw)
            rospy.loginfo("<Gyro-Acc>     (Surg,Sway,Heav) = (%02.3f, %02.3f, %02.3f)", self.__gyro.acc_surge, self.__gyro.acc_sway, self.__gyro.acc_heave)

            print("--------------------------Actuator---------------------------")
            if self.__autosw.data == 1:
                rospy.loginfo("#--   AUTO  --#")
                rospy.loginfo("Motor:  [PWR]%d%% [ROT]%drpm [AMP]%.1fA [TMP]%.1fC", self.__automotor1.data, self.__motor1_info.rpm, self.__motor1_info.current, self.__motor1_info.temperature)
                rospy.loginfo("Rudder: [PWR]%d%% [ROT]%.1f Deg", self.__autorudder1.data, self.__rudder1_info.data)
            else:
                rospy.loginfo("#--  MANUAL --#")
                rospy.loginfo("Motor:  [PWR]%d%% [ROT]%drpm [AMP]%.1fA [TMP]%.1fC", self.__rcmotor1.data, self.__motor1_info.rpm, self.__motor1_info.current, self.__motor1_info.temperature)
                rospy.loginfo("Rudder: [PWR]%d%% [ROT]%.1f Deg", self.__rcrudder1.data, self.__rudder1_info.data)


            
            print("\n\n\n")
            self.locker.release()
            r.sleep()

            
    def auto_watchflag(self):
        i = 0
        
        while not rospy.is_shutdown():

            self.locker.acquire()
            if self.__autoflag == 0 and self.__autosw.data == 1:
                self.__autoflag = 1
                self.locker.release()
                try:
                    th_genmap = threading.Thread(target = self.auto_genmap, args = (i,))
                    th_graph = threading.Thread(target = self.auto_graph, args = (i,))                    
                    th_genmap.start()
                    th_graph.start()
                except:
                    pass


            elif self.__autoflag == 1 and self.__autosw.data == 0:
                self.__autoflag = 0
                self.locker.release()
                i = i+1

            else:
                self.locker.release()
                

    def auto_genmap(self, cnt):
        r = rospy.Rate(self.FILELOG_RATE)

        filename = self.__filename + 'map'+'_test' + str(cnt) + '.csv'
        file = open(filename, 'w')

        latitude = 0.0
        latitude_dir = ''
        longitude = 0.0
        longitude_dir = ''
        
        self.locker.acquire()
        
        while self.__autosw.data == 1 and not rospy.is_shutdown():
            latitude = self.__gpspos.latitude_deg + self.__gpspos.latitude_min/60.0
            longitude = self.__gpspos.longitude_deg + self.__gpspos.longitude_min/60.0
            latitude_dir = self.__gpspos.latitude_dir
            longitude_dir = self.__gpspos.longitude_dir
            self.locker.release()

            if latitude_dir == 'S':
                latitude = -latitude

            if longitude_dir == 'W':
                longitude = -longitude
            

            file.write(str(latitude)+','+str(longitude)+'\n')
            r.sleep()
            self.locker.acquire()

        self.locker.release()
        file.close()


    def auto_graph(self, cnt):
        r = rospy.Rate(self.FILELOG_RATE)

        filename = self.__filename + 'graph'+'_test' + str(cnt) + '.csv'
        file = open(filename, 'w')

        yaw = 0.0
        delta = 0.0
        rudder_pwr = 0
        hdg = 0.0

        starttime = time.time()
        self.locker.acquire()
        
        while self.__autosw.data == 1 and not rospy.is_shutdown():
            current_time = time.time()
            yaw = self.__gyro.ang_yaw
            delta = self.__rudder1_info.data
            rudder_pwr = self.__autorudder1.data
            hdg = self.__gpsnav.truenorth_heading
            self.locker.release()
            
            file.write("{0:.2f}".format(current_time - starttime)+','+str(yaw)+','+str(delta)+','+str(rudder_pwr)+','+str(hdg)+'\n')
            r.sleep()
            self.locker.acquire()

        self.locker.release()
        file.close()
    

    def filelog(self):
        r = rospy.Rate(self.FILELOG_RATE)

        filename = self.__filename + 'log.csv'
        file = open(filename, 'w')

        self.locker.acquire()
        logarr = [ ['date', str(self.__gpstime.year)+'/'+str(self.__gpstime.month)+'/'+str(self.__gpstime.day)],
                   ['time', str(self.__gpstime.hour)+':'+str(self.__gpstime.min)+':'+str(self.__gpstime.sec)],
                   ['rc_autosw', str(self.__autosw.data)],
                   ['rc_motor1', str(self.__rcmotor1.data)],
                   ['rc_rudder1', str(self.__rcrudder1.data)],
                   ['auto_motor1', str(self.__automotor1.data)],
                   ['auto_rudder1', str(self.__autorudder1.data)],
                   ['motor1_rot', str(self.__motor1_info.rpm)],
                   ['motor1_current', str(self.__motor1_info.current)],
                   ['motor1_temperature', str(self.__motor1_info.temperature)],
                   ['rudder1_angle', str(self.__rudder1_info.data)],
                   ['gyro_ang_roll', str(self.__gyro.ang_roll)],
                   ['gyro_ang_pitch', str(self.__gyro.ang_pitch)],
                   ['gyro_ang_yaw', str(self.__gyro.ang_yaw)],
                   ['gyro_vel_roll', str(self.__gyro.vel_roll)],
                   ['gyro_vel_pitch', str(self.__gyro.vel_pitch)],
                   ['gyro_vel_yaw', str(self.__gyro.vel_yaw)],
                   ['gyro_acc_surge', str(self.__gyro.acc_surge)],
                   ['gyro_acc_sway', str(self.__gyro.acc_sway)],
                   ['gyro_acc_heave', str(self.__gyro.acc_heave)],
                   ['gps_quality', str(self.__gpspos.gps_quality)],
                   ['gps_satellite_number', str(self.__gpspos.satellite_number)],
                   ['gps_pos_latitude_dir', self.__gpspos.latitude_dir],
                   ['gps_pos_latitude',str(self.__gpspos.latitude_deg + self.__gpspos.latitude_min/60.0)],
                   ['gps_pos_latitude_error', str(self.__gpspos.latitude_err)],
                   ['gps_pos_longitude_dir', self.__gpspos.longitude_dir],
                   ['gps_pos_longitude', str(self.__gpspos.longitude_deg + self.__gpspos.longitude_min/60.0)],
                   ['gps_pos_longitude_error', str(self.__gpspos.longitude_err)],
                   ['gps_loc_x', str(self.__gpsloc.localpos_e)],
                   ['gps_loc_y', str(self.__gpsloc.localpos_n)],
                   ['gps_loc_z', str(self.__gpsloc.localpos_z)],
                   ['gps_loc_phi', str(self.__gpsloc.local_azimath)],
                   ['gps_loc_theta', str(self.__gpsloc.local_vertical)],
                   ['gps_loc_range', str(self.__gpsloc.local_range)],
                   ['speed_kt', str(self.__gpsnav.speed_knot)],
                   ['truenorth_hdg', str(self.__gpsnav.truenorth_heading)],
                   ['truenorth_heading_error', str(self.__gpsnav.truenorth_heading_err)],
        ]
        self.locker.release()
        
        for i in logarr:
            file.write(i[0])
            file.write(',')
        file.write("\n")
        
        while not rospy.is_shutdown():
            
            self.locker.acquire()
            logarr = [ ['date', str(self.__gpstime.year)+'/'+str(self.__gpstime.month)+'/'+str(self.__gpstime.day)],
                       ['time', str(self.__gpstime.hour)+':'+str(self.__gpstime.min)+':'+str(self.__gpstime.sec)],
                       ['rc_autosw', str(self.__autosw.data)],
                       ['rc_motor1', str(self.__rcmotor1.data)],
                       ['rc_rudder1', str(self.__rcrudder1.data)],
                       ['auto_motor1', str(self.__automotor1.data)],
                       ['auto_rudder1', str(self.__autorudder1.data)],
                       ['motor1_rot', str(self.__motor1_info.rpm)],
                       ['motor1_current', str(self.__motor1_info.current)],
                       ['motor1_temperature', str(self.__motor1_info.temperature)],
                       ['rudder1_angle', str(self.__rudder1_info.data)],
                       ['gyro_ang_roll', str(self.__gyro.ang_roll)],
                       ['gyro_ang_pitch', str(self.__gyro.ang_pitch)],
                       ['gyro_ang_yaw', str(self.__gyro.ang_yaw)],
                       ['gyro_vel_roll', str(self.__gyro.vel_roll)],
                       ['gyro_vel_pitch', str(self.__gyro.vel_pitch)],
                       ['gyro_vel_yaw', str(self.__gyro.vel_yaw)],
                       ['gyro_acc_surge', str(self.__gyro.acc_surge)],
                       ['gyro_acc_sway', str(self.__gyro.acc_sway)],
                       ['gyro_acc_heave', str(self.__gyro.acc_heave)],
                       ['gps_quality', str(self.__gpspos.gps_quality)],
                       ['gps_satellite_number', str(self.__gpspos.satellite_number)],
                       ['gps_pos_latitude_dir', self.__gpspos.latitude_dir],
                       ['gps_pos_latitude',str(self.__gpspos.latitude_deg + self.__gpspos.latitude_min/60.0)],
                       ['gps_pos_latitude_error', str(self.__gpspos.latitude_err)],
                       ['gps_pos_longitude_dir', self.__gpspos.longitude_dir],
                       ['gps_pos_longitude', str(self.__gpspos.longitude_deg + self.__gpspos.longitude_min/60.0)],
                       ['gps_pos_longitude_error', str(self.__gpspos.longitude_err)],
                       ['gps_loc_x', str(self.__gpsloc.localpos_e)],
                       ['gps_loc_y', str(self.__gpsloc.localpos_n)],
                       ['gps_loc_z', str(self.__gpsloc.localpos_z)],
                       ['gps_loc_phi', str(self.__gpsloc.local_azimath)],
                       ['gps_loc_theta', str(self.__gpsloc.local_vertical)],
                       ['gps_loc_range', str(self.__gpsloc.local_range)],
                       ['speed_kt', str(self.__gpsnav.speed_knot)],
                       ['truenorth_hdg', str(self.__gpsnav.truenorth_heading)],
                       ['truenorth_heading_error', str(self.__gpsnav.truenorth_heading_err)],
            ]
            self.locker.release()
            
            for i in logarr:
                file.write(i[1])
                file.write(',')
            file.write("\n")
                
            r.sleep()

        file.close()


#Main-----------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        log = LoggerClass()

    except rospy.ROSInterruptException:
        pass
