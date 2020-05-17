#include "ros/ros.h"
#include "shipcon/gyro.h"
#include "std_msgs/Int16.h"
#include "boost/thread.hpp"

#include <stdio.h>
#include <stdlib.h> /*exit()*/
#include <termios.h>
#include <string.h> /*memset()*/
#include <fcntl.h> /*open(), O_RDWR*/
#include <termios.h>
#include <unistd.h> /*read()*/

#define DEV_NAME "/dev/ttyUSB0"
#define BAUD_RATE_IN B9600
#define BAUD_RATE_OUT B9600
#define BUFF_SIZE 70
#define PUBLISH_RATE 10 //Hz

const char GYRO_START[6] = {0x10, 0x02, 0x01, 0x01, 0x10, 0x03};
const char GYRO_REQ_STAT[8] = {0x10, 0x02, 0x10, 0x10, 0x10, 0x10, 0x10, 0x03};
const char GYRO_RESET[6] = {0x10, 0x02, 0x08, 0x08, 0x10, 0x03};
const char GYRO_OFFSET_CANCEL[6] = {0x10, 0x02, 0x80, 0x80, 0x10, 0x03};



class GyroClass{

public:
  /** Constructor **/
  GyroClass(){
	/* Initialize Varibles */
	fd_ = 0;
	
	/*Initialize Serial Port*/
	fd_ = open(DEV_NAME, O_RDWR);
	if(fd_<0){/*READ WRITE ENABLE*/
	  ROS_INFO("##ERROR##  Failed to detect Gyro! Check USB Connection!!");
	  exit(-1);
	}
	ROS_INFO("Succeeded to open "DEV_NAME"");
	serial_init(fd_);

	/*Initialize ROS*/
	pub_ = nh_.advertise<shipcon::gyro>("gyro", 1);
	rc_zero_sub_ = nh_.subscribe("rc_gyrosw", 1, &GyroClass::rcreset_cb, this);
	auto_zero_sub_ = nh_.subscribe("auto_gyrosw", 10, &GyroClass::autoreset_cb, this);

	/* Start Gyro */
	send_stx();

	/* Start Thread */
	boost::thread th_ref( boost::bind(&GyroClass::refresh, this) );
	boost::thread th_pub( boost::bind(&GyroClass::publish, this) );
	
  }
  
  /** Destructor **/
  ~GyroClass(){
	;
  }

 
private:
  /** Class Variables **/
  int fd_;
  shipcon::gyro gyro_msg_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber rc_zero_sub_;
  ros::Subscriber auto_zero_sub_;

  /** Class Instance **/
  boost::mutex mtx_;

  /** Class Method **/
  void serial_init(int fd){
	struct termios tio;
	memset(&tio, 0, sizeof(tio));
	tio.c_cflag = CS8 | CLOCAL | CREAD | PARENB; /*8bit, local, read_enable, even_parity*/
	tio.c_cc[VTIME] = 100; /*VMIN:minimum data size, VTIME:timeout(Reset with VMIN)*/
	tio.c_cc[VMIN] = 0;
	
	cfsetispeed(&tio, BAUD_RATE_IN);
	cfsetospeed(&tio, BAUD_RATE_OUT);

	tcsetattr(fd, TCSANOW, &tio);
  }

  void send_stx(void){
	write(fd_, GYRO_START, sizeof(GYRO_START));
	ROS_INFO("####Sent Start Signal...####");
  }

  void zero_reset(void){
	write(fd_, GYRO_RESET, sizeof(GYRO_RESET));
	ROS_INFO("####Sent Reset Yaw Angle Signal...####");
  }

  void autoreset_cb(const std_msgs::Int16 msg){
	if(msg.data == 1){
	  zero_reset();
	}
  }

  void rcreset_cb(const std_msgs::Int16 msg){
	if(msg.data == 1){
	  zero_reset();
	}
  }

  void refresh(void){
	/* Local Variables */
	int checksum = 0;
	int status = 0;
	
	unsigned char data[40];
	unsigned char buf = 0x00;

	short tmp_data[3] = {0,0,0};
	shipcon::gyro msg;
	int i;
	int j;
	int len;
	bool dle = false;


	while(ros::ok()){//main

	  /*Get and modify data*/
	  read(fd_, &buf, 1);

	  if(dle == false){
		if(buf == 0x10){//Detct DLE
		  dle = true;
		}

	  }else if(buf == 0x02){//Detct DLE+STX
		dle = false;
		i = 0;

		while(1){
		  if(read(fd_, &buf, 1)){//if read nothing, this sequence will be skipped
			
			if(dle == true){//Judging DLE+DLE or DLE+ETX
			  if(buf == 0x10){
				data[i] = 0x10;
			  }else if(buf == 0x03){
				break;
			  }
			  

			}else{//Detect DLE or substitute data
			  if(buf == 0x10){
				dle = true;
			  }else{
				data[i] = buf;
			  }
			}
			
			i++;
		  

		  }/*END:  if(read(fd, &buf, 1))*/
		
		}/*END:   while(1)*/

		status = data[0];

		for(j=0;j<3;j++){
		  tmp_data[j] = ((data[j*2+1]<<7) + (data[j*2+2]>>1)) * 2;
		}
		msg.ang_roll = -(tmp_data[0] * 180.0) / 32767;
		msg.ang_pitch = -(tmp_data[1] * 180.0) / 32767;
		msg.ang_yaw = -(tmp_data[2] * 180.0) / 32767;

		for(j=0;j<3;j++){
		  tmp_data[j] = ((data[j*2+7]<<7) + (data[j*2+8]>>1)) * 2;
		}
		msg.vel_roll = -(tmp_data[0] * 200.0) / 32767;
		msg.vel_pitch = -(tmp_data[1] * 200.0) / 32767;
		msg.vel_yaw = -(tmp_data[2] * 200.0) / 32767;

		for(j=0;j<3;j++){
		  tmp_data[j] = ((data[j*2+13]<<7) + (data[j*2+14]>>1)) * 2;
		}
		msg.acc_surge = -(tmp_data[0] * 12.0) / 32767;
		msg.acc_sway = -(tmp_data[1] * 12.0) / 32767;
		msg.acc_heave = -(tmp_data[2] * 12.0) / 32767;


		/* Substitute Class Varibales */
		mtx_.lock();
		gyro_msg_ = msg;
		mtx_.unlock();
	  

	  }/*END: if(dle == false)*/
	  
	}/*END: while(ros::ok())*/
  }//END: This Method

  void publish(void){
	ros::Rate loop_rate(PUBLISH_RATE);
	
	while(ros::ok()){
	  mtx_.lock();
	  pub_.publish(gyro_msg_);
	  ROS_INFO("[ANG] ROLL :%f, PITCH:%f, YAW  :%f",gyro_msg_.ang_roll, gyro_msg_.ang_pitch, gyro_msg_.ang_yaw);
	  ROS_INFO("[VEL] ROLL :%f, PITCH:%f, YAW  :%f",gyro_msg_.vel_roll, gyro_msg_.vel_pitch, gyro_msg_.vel_yaw);
	  ROS_INFO("[ACC] SURGE:%f, SWAY :%f, HEAVE:%f\n",gyro_msg_.acc_surge, gyro_msg_.acc_sway, gyro_msg_.acc_heave);
	  mtx_.unlock();

	  loop_rate.sleep();
	}
	
  }//END: This Method

  
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "NODE_Gyro");
	GyroClass gyro;
	ros::spin();
}
  
