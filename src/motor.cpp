#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "shipcon/motor_info.h"
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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//#include <mutex>

#define MOTOR_IP "192.168.39.10"
#define MOTOR_PORT 50000

class MotorUDPComClass{
public:
  /** Constructor **/
  MotorUDPComClass(){
	init_ethernet();
	pub_status_ = nh_.advertise<shipcon::motor_info>("motor1_info", 1);
	pub_error_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("motor1_error", 1);
	sub_ctrl_ = nh_.subscribe("auto_motor1", 1, &MotorUDPComClass::ctrl_value_sub_callback, this);
	msg_error_.name = "Motor_info_error";
	ctrl_value_msg_.data = 0;

	boost::thread thread_sub(boost::bind(&MotorUDPComClass::send_signal_thread, this));
	boost::thread thread_pub(boost::bind(&MotorUDPComClass::publish_info_thread, this));
	boost::thread thread_refresh(boost::bind(&MotorUDPComClass::refresh_info_thread, this));
  }

  /** Destrunctor **/
  ~MotorUDPComClass(){

  }
  
private:
  /** Variables - Instances **/
  //Thread
  boost::mutex mtx_;
  
  //Ethernet
  int sock_;
  struct sockaddr_in addr_;

  //ROS
  ros::NodeHandle nh_;
  ros::Publisher pub_status_; 
  ros::Publisher pub_error_;
  ros::Subscriber sub_ctrl_;
  shipcon::motor_info msg_status_;
  diagnostic_msgs::DiagnosticStatus msg_error_;
  std_msgs::Int16 ctrl_value_msg_;
  

  /** Methods **/
  //UDP
  void init_ethernet(void){
	sock_ = socket(AF_INET, SOCK_DGRAM, 0);
	addr_.sin_family = AF_INET;
	addr_.sin_port = htons(50001);
	addr_.sin_addr.s_addr = INADDR_ANY;
	bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_));
  }

  int recv_udp(std::string ip, char* data, const int data_length){
	/*Declare and Initialize Local Variables*/
	memset(data, '\0', data_length);
	struct sockaddr_in addr_src;
	int addr_src_len = sizeof(sockaddr_in);
	std::string recv_ip_addr;
	int recv_size = 0;
	
	/*Receive data via UDP*/
	recv_size = recvfrom(sock_, data, data_length, 0, (struct sockaddr *)&addr_src, (socklen_t *)&addr_src_len);
	if(recv_size==-1){
	  return -1;
	}

	/*Evaluate Source IP*/
	recv_ip_addr = inet_ntoa(addr_src.sin_addr);
	if(recv_ip_addr == ip){
	  return recv_size;
	}else{
	  memset(data, 0, sizeof(data));
	  return 0;
	}
	  
  }

  int send_udp(int port, char* ip, char* data, const int data_length){
	struct sockaddr_in addr_dest;
	int test;

	addr_dest.sin_family = AF_INET;
	addr_dest.sin_port = htons(port);
	addr_dest.sin_addr.s_addr = inet_addr(ip);
	
	return sendto(sock_, data, data_length, 0, (struct sockaddr *)&addr_dest, sizeof(sockaddr_in));
  }

  //ROS
  void ctrl_value_sub_callback(const std_msgs::Int16 value){
	char buffer[4];
	char dest_ip[] = MOTOR_IP;
	int send_size;

	memset(buffer, '\0', sizeof(buffer));
	memcpy(buffer, &value, sizeof(buffer));
	send_size = send_udp(MOTOR_PORT, dest_ip, buffer, sizeof(buffer));
	mtx_.lock();
	ROS_INFO("Sent:%d Byte", send_size);
	mtx_.unlock();
  }

  void refresh_info_thread(){
	ros::Rate loop_rate(10);

	while(ros::ok()){
		mtx_.lock();
		pub_status_.publish(msg_status_);
		pub_error_.publish(msg_error_);
		ROS_INFO("%d[rpm]  %.1f[A]  %.1f[DegCel]", msg_status_.rpm, msg_status_.current, msg_status_.temperature);
		mtx_.unlock();	  

		loop_rate.sleep();
	}
  }


  void publish_info_thread(){
	char buffer[10];
	
	while(ros::ok()){
	  if(recv_udp(MOTOR_IP, buffer, sizeof(buffer))>0){
		mtx_.lock();
		msg_error_.level = msg_error_.OK;
		msg_error_.message = "";
		memcpy(&(msg_status_.rpm), &buffer[0], 2);
		memcpy(&(msg_status_.current), &buffer[2], 4);
		memcpy(&(msg_status_.temperature), &buffer[6], 4);
		mtx_.unlock();

	  }else{
		mtx_.lock();
		msg_error_.level = msg_error_.ERROR;
		msg_error_.message = "Missing_Motor_Info";
		msg_status_.rpm = 0;
		msg_status_.current = 0.0;
		msg_status_.temperature = 0.0;
		mtx_.unlock();

	  }

	}
  }

  void send_signal_thread(){
  
 	while(ros::ok()){
	  ;
	}
  }



};


int main(int argc, char *argv[]){
  ros::init(argc, argv, "NODE_Motor1");
  MotorUDPComClass motor_udp;
  ros::spin();
  return 0;
}


	
