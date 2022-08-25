#include "ros/ros.h"
#include "std_msgs/Float32.h"/*Rudder Angle*/
#include "std_msgs/Int16.h"/*Thruster,RC*/
#include "shipcon/gyro.h" /*Gyro*/
#include "shipcon/gps_navinfo.h" /*GPS -SPD- -HDG-*/
#include "shipcon/gps_position.h"
#include "shipcon/gps_local_position.h"
#include "shipcon/gps_time.h"

#include "boost/thread.hpp"
#include <stdio.h>
#include <stdlib.h> /*exit()*/
#include <termios.h>
#include <string.h> /*memset()*/
#include <fcntl.h> /*open(), O_RDWR*/
#include <termios.h>
#include <unistd.h> /*read()*/
#include <errno.h> /*=int errno;*/
#include <sys/types.h>
#include <time.h>

/*** Constants ***/


class LoggerClass{
public:
  /** Constructor **/
  LoggerClass(){
 
	sub_gyro_ = nh_.subscribe("Gyro", 5, &LoggerClass::gyro_cb, this);
	sub_gps_pos_ = nh_.subscribe("GPS_Position", 5, &LoggerClass::gps_position_cb, this);
	sub_gps_time_ = nh_.subscribe("Local_Position", 5, &LoggerClass::gps_time_cb, this);
	sub_gps_navinfo_ = nh_.subscribe("Navigation_Info", 5, &LoggerClass::gps_navinfo_cb, this);
	sub_gps_local_position_ = nh_.subscribe("GPS_Time", 5, &LoggerClass::gps_local_position_cb, this);
	sub_rc_throttle_ = nh_.subscribe("Main_Thruster", 5, &LoggerClass::rc_throttle_cb, this);
	sub_rc_rudder_ = nh_.subscribe("Main_Rudder", 5, &LoggerClass::rc_rudder_cb, this);
	sub_rc_automode_ = nh_.subscribe("AutoMode", 5, &LoggerClass::rc_automode_cb, this);
	sub_motor_power_ = nh_.subscribe("Automotor_1", 5, &LoggerClass::motor_power_cb, this);
	sub_motor_info_ = nh_.subscribe("Motor_Status_Info", 5, &LoggerClass::motor_info_cb, this);
	sub_rudder_power_ = nh_.subscribe("Autorudder_1", 5, &LoggerClass::rudder_power_cb, this);
	sub_rudder_angle_ = nh_.subscribe("Rudder_Angle", 5, &LoggerClass::rudder_angle_cb, this);

	automode_en_ = 0;
  }

  

  /** Destrunctor **/
  ~LoggerClass(){

  }
  
private:
  /** Variables - Instances **/
  //Thread
  boost::mutex mtx_;

  //ROS Node
  ros::NodeHandle nh_;
  ros::Subscriber sub_gyro_; //shipcon::gyro
  ros::Subscriber sub_gps_pos_; //shipcon::gps_position
  ros::Subscriber sub_gps_time_; //shipcon::gps_time
  ros::Subscriber sub_gps_navinfo_; //shipcon::gps_navinfo
  ros::Subscriber sub_gps_local_position_; //shipcon::gps_local_position
  ros::Subscriber sub_rc_throttle_; //std_msgs::Int16
  ros::Subscriber sub_rc_rudder_; //std_msgs::Int16
  ros::Subscriber sub_rc_automode_; //std_msgs::Int16
  ros::Subscriber sub_motor_power_; //std_msgs::Int16
  ros::Subscriber sub_motor_info_; //std_msgs::Int16
  ros::Subscriber sub_rudder_power_; //std_msgs::Int16
  ros::Subscriber sub_rudder_angle_; //std_msgs::Float32

  //ROS Message
  shipcon::gyro msg_gyro_; 
  shipcon::gps_position msg_gps_pos_;
  shipcon::gps_time msg_gps_time_;
  shipcon::gps_navinfo msg_gps_navinfo_;
  shipcon::gps_local_position msg_gps_local_position_;
  std_msgs::Int16 msg_rc_throttle_;
  std_msgs::Int16 msg_rc_rudder_;
  std_msgs::Int16 msg_rc_automode_;
  std_msgs::Int16 msg_motor_power_;
  std_msgs::Int16 msg_motor_info_;
  std_msgs::Int16 msg_rudder_power_;
  std_msgs::Float32 msg_rudder_angle_;

  int automode_en_;
  
  /** Methods **/
  //ROS
  void gyro_cb(const shipcon::gyro msg){
	mtx_.lock();
	msg_gyro_ = msg;
	mtx_.unlock();
  }
  
  void gps_position_cb(const shipcon::gps_position msg){
	mtx_.lock();
	msg_gps_pos_ = msg;
	mtx_.unlock();
  }
  
  void gps_time_cb(const shipcon::gps_time msg){
	mtx_.lock();
	msg_gps_time_ = msg;
	mtx_.unlock();
  }
  
  void gps_navinfo_cb(const shipcon::gps_navinfo msg){
	mtx_.lock();
	msg_gps_navinfo_ = msg;
	mtx_.unlock();
  }
  
  void gps_local_position_cb(const shipcon::gps_local_position msg){
	mtx_.lock();
	msg_gps_local_position_ = msg;
	mtx_.unlock();
  }
  
  void rc_throttle_cb(const std_msgs::Int16 msg){
	mtx_.lock();
	msg_rc_throttle_ = msg;
	mtx_.unlock();
  }
  
  void rc_rudder_cb(const std_msgs::Int16 msg){
	mtx_.lock();
	msg_rc_rudder_ = msg;
	mtx_.unlock();
  }
  
  void rc_automode_cb(const std_msgs::Int16 msg){
	mtx_.lock();
	msg_rc_automode_ = msg;
	mtx_.unlock();

	if(automode_en_ == 0 && msg.data == 1){
	  boost::thread thread_log(boost::bind(&LoggerClass::log_thread, this));
	  automode_en_ = msg.data;
	}
	
  }
  
  void motor_power_cb(const std_msgs::Int16 msg){
	mtx_.lock();
	msg_motor_power_ = msg;
	mtx_.unlock();
  }
  
  void motor_info_cb(const std_msgs::Int16 msg){
	mtx_.lock();
	msg_motor_info_ = msg;
	mtx_.unlock();
  }
  
  void rudder_power_cb(const std_msgs::Int16 msg){
	mtx_.lock();
	msg_rudder_power_ = msg;
	mtx_.unlock();
  }
  
  void rudder_angle_cb(const std_msgs::Float32 msg){
	mtx_.lock();
	msg_rudder_angle_ = msg;
	mtx_.unlock();
  }


  void log_thread(){
	ros::Rate loop_rate(10);
	shipcon::gyro localmsg_gyro_; 
	shipcon::gps_position localmsg_gps_pos_;
	shipcon::gps_time localmsg_gps_time_;
	shipcon::gps_navinfo localmsg_gps_navinfo_;
	shipcon::gps_local_position localmsg_gps_local_position_;
	std_msgs::Int16 localmsg_rc_throttle_;
	std_msgs::Int16 localmsg_rc_rudder_;
	std_msgs::Int16 localmsg_rc_automode_;
	std_msgs::Int16 localmsg_motor_power_;
	std_msgs::Int16 localmsg_motor_info_;
	std_msgs::Int16 localmsg_rudder_power_;
	std_msgs::Float32 localmsg_rudder_angle_;
	double latitude;
	std::string lat_dir;
	float longitude;
	std::string lon_dir;

	FILE *log_fp;
	time_t local_time;
	char filename[64];
	
	local_time = time(NULL);
	strftime(filename, sizeof(filename), "~/Experiment_%Y%m%d_%H%M%S.csv", localtime(&local_time));
	if( (log_fp = fopen(filename, "w")) == NULL ){
	  exit(EXIT_FAILURE);
	}
	fprintf(log_fp, "GPSdate");
	fprintf(log_fp, "GPStime");

	while(ros::ok() && automode_en_ == 1){
	  mtx_.lock();
	  localmsg_gyro_ = msg_gyro_; 
	  localmsg_gps_pos_ = msg_gps_pos_;
	  localmsg_gps_time_ = msg_gps_time_;
	  localmsg_gps_navinfo_ = msg_gps_navinfo_;
	  localmsg_gps_local_position_ = msg_gps_local_position_;
	  localmsg_rc_throttle_ = msg_rc_throttle_;
	  localmsg_rc_rudder_ = msg_rc_rudder_;
	  localmsg_rc_automode_ = msg_rc_automode_;
	  localmsg_motor_power_ = msg_motor_power_;
	  localmsg_motor_info_ = msg_motor_info_;
	  localmsg_rudder_power_ = msg_rudder_power_;
	  localmsg_rudder_angle_ = msg_rudder_angle_;
	  mtx_.unlock();

	  
	  lat_dir = localmsg_gps_pos_.latitude_dir.c_str();
	  latitude = localmsg_gps_pos_.latitude_deg + localmsg_gps_pos_.latitude_min /60.0;
	  if(lat_dir == "N"){
		;
	  }else if(lat_dir == "S"){
		latitude = -latitude;
	  }else{
		latitude = -1000.0;
	  }
	  
	  lon_dir = localmsg_gps_pos_.longitude_dir.c_str();
	  longitude = localmsg_gps_pos_.longitude_deg + localmsg_gps_pos_.longitude_min /60.0;
	  if(lon_dir == "E"){
		;
	  }else if(lon_dir == "W"){
		longitude = -longitude;
	  }else{
		longitude = -1000.0;
	  }

	  fprintf(log_fp, "%d/%d/%d,", localmsg_gps_time_.year, localmsg_gps_time_.month, localmsg_gps_time_.day);
	  fprintf(log_fp, "%d:%d:%d,", localmsg_gps_time_.hour, localmsg_gps_time_.min, localmsg_gps_time_.sec);
	  fprintf(log_fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,", localmsg_gyro_.ang_roll, localmsg_gyro_.ang_pitch, localmsg_gyro_.ang_yaw, localmsg_gyro_.vel_roll, localmsg_gyro_.vel_pitch, localmsg_gyro_.vel_yaw, localmsg_gyro_.acc_surge, localmsg_gyro_.acc_sway, localmsg_gyro_.acc_heave);
	  fprintf(log_fp, "%lf,%lf", latitude, longitude);
	  loop_rate.sleep();

	}
  }



};


int main(int argc, char *argv[]){
  ros::init(argc, argv, "Log_Node");
  LoggerClass logger;
  ros::spin();
  return 0;
}


	
