#include "ros/ros.h"
#include "std_msgs/Float32.h"/*Rudder Angle*/
#include "std_msgs/Int16.h"/*Thruster,RC*/
#include "shipcon/gyro.h" /*Gyro*/
#include "shipcon/gps_navinfo.h" /*GPS -SPD- -HDG-*/
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

#define Z_HDG 20 //deg
#define Z_DELTA 20 //deg
#define MOTOR_POWER 40 //%

class ZNavClass{
public:
  /** Constructor **/
  ZNavClass(){
	
	pub_thruster_ = nh_.advertise<std_msgs::Int16>("auto_motor1", 1);
	pub_rudder_ = nh_.advertise<std_msgs::Int16>("auto_rudder1", 1);
	pub_gyrosw_ = nh_.advertise<std_msgs::Int16>("auto_gyrosw" , 10);
	sub_gyro_ = nh_.subscribe("gyro", 1, &ZNavClass::refresh_gyro_callback, this);
	sub_gps_ = nh_.subscribe("gps_position", 1, &ZNavClass::refresh_gps_callback, this);
	sub_rc_ = nh_.subscribe("rc_autosw", 1, &ZNavClass::refresh_rc_callback, this);

	msg_gyrosw_.data = 1;

	boost::thread thread_pub(boost::bind(&ZNavClass::publish_actuator_thread, this));

  }

  /** Destrunctor **/
  ~ZNavClass(){

  }
  
private:
  /** Variables - Instances **/
  //Thread
  boost::mutex mtx_;

  //ROS Node
  ros::NodeHandle nh_;
  ros::Subscriber sub_gyro_;
  ros::Subscriber sub_gps_;
  ros::Subscriber sub_rc_;
  ros::Publisher pub_thruster_;
  ros::Publisher pub_rudder_;
  ros::Publisher pub_gyrosw_;

  //ROS Message
  std_msgs::Float32 msg_delta_;
  std_msgs::Int16 msg_motor_;
  std_msgs::Int16 msg_rudder_;
  shipcon::gyro msg_gyro_;
  shipcon::gps_navinfo msg_gps_;
  std_msgs::Int16 msg_automode_;
  std_msgs::Int16 msg_gyrosw_;

  /** Methods **/
  //ROS
  void refresh_gyro_callback(const shipcon::gyro msg){
	mtx_.lock();
	msg_gyro_ = msg;
	ROS_INFO("Gyro-Yaw:[Ang]%.2f deg\t[Vel]%.2f deg/s", msg_gyro_.ang_yaw, msg_gyro_.vel_yaw);
	mtx_.unlock();
  }

  void refresh_gps_callback(const shipcon::gps_navinfo msg){
	mtx_.lock();
	msg_gps_ = msg;
	ROS_INFO("GPS:%.2f kt\t %.2f HDG", msg_gps_.speed_knot, msg_gps_.truenorth_heading);
	mtx_.unlock();
  }

  void refresh_rc_callback(const std_msgs::Int16 msg){
	mtx_.lock();
	msg_automode_ = msg;
	mtx_.unlock();
  }

  void refresh_actuator_thread(){
	float gyro_yaw = 0.0;
	int motor = 0;
	int rudder = 0;
	int turn_mode = 1; /*STBD:+1 PORT:-1*/
 
	while(ros::ok()){
	  //Evaluate Automode or not
	  mtx_.lock();
	  if(msg_automode_.data != 1){
		mtx_.unlock();
		break;
	  }
	  mtx_.unlock();	  

	  //Get Values
	  mtx_.lock();
	  gyro_yaw = msg_gyro_.ang_yaw;
	  mtx_.unlock();

	  //Generate Z Motion
	  if(turn_mode == 1){
		if(gyro_yaw > Z_HDG){
		  mtx_.lock();
		  msg_rudder_.data = (Z_DELTA*100) / 60;
		  mtx_.unlock();
		  turn_mode = -1;
		}else{
		  mtx_.lock();
		  msg_rudder_.data = (-Z_DELTA*100) / 60;
		  mtx_.unlock();
		}
	  }else if(turn_mode == -1){		  
		if(gyro_yaw < -Z_HDG){
		  mtx_.lock();
		  msg_rudder_.data = (-Z_DELTA*100) / 60;
		  mtx_.unlock();
		  turn_mode = 1;
		}else{
		  mtx_.lock();
		  msg_rudder_.data = (Z_DELTA*100) / 60;
		  mtx_.unlock();
		}
	  }

	  //Generate Propeller Rotation
	  mtx_.lock();
	  msg_motor_.data = MOTOR_POWER;
	  mtx_.unlock();

	  //Generate Message
	  mtx_.lock();
	  //ROS_INFO("[Target] Rudder:%d, Motor:%d / [Cuurent] Yaw:%.3f", msg_rudder_.data, msg_motor_.data, gyro_yaw);
	  mtx_.unlock();

	}


		
  }

  void publish_actuator_thread(){
	ros::Rate loop_rate(10);
	int auto_enable = 0;
	
	mtx_.lock();
	auto_enable = msg_automode_.data;
	mtx_.unlock();

	while(ros::ok()){
	  //Evaluate SW
	  if(auto_enable == 0){
		
		mtx_.lock();
		if(msg_automode_.data == 1){
		  mtx_.unlock();
		  auto_enable = 1;

		  boost::thread thread_act(boost::bind(&ZNavClass::refresh_actuator_thread, this));
		  ROS_INFO("Auto Thtead Started");
		  mtx_.lock();
		  pub_thruster_.publish(msg_motor_);
		  pub_rudder_.publish(msg_rudder_);
		  pub_gyrosw_.publish(msg_gyrosw_);
		  mtx_.unlock();


		}else{
		  mtx_.unlock();
		  continue;
		}
	  }else if(auto_enable == 1){
		mtx_.lock();
		if(msg_automode_.data == 1){
		  pub_thruster_.publish(msg_motor_);
		  pub_rudder_.publish(msg_rudder_);
		  mtx_.unlock();
		}else{
		  mtx_.unlock();
		  auto_enable = 0;
		  continue;
		}
	  }
	  loop_rate.sleep();

	}
  }



};


int main(int argc, char *argv[]){
  ros::init(argc, argv, "NODE_Auto");
  ZNavClass main_navi;
  ros::spin();
  return 0;
}


	
