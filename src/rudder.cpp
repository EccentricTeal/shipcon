#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
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

#define RUDDER_IP "192.168.39.11"
#define RUDDER_PORT 50000

class RudderUDPComClass{
public:
  /** Constructor **/
  RudderUDPComClass(){
	init_ethernet();
	pub_angle_ = nh_.advertise<std_msgs::Float32>("rudder1_info", 1);
	pub_error_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("rudder1_error", 1);
	sub_ctrl_ = nh_.subscribe("auto_rudder1", 1, &RudderUDPComClass::ctrl_value_sub_callback, this);
	msg_error_.name = "Rudder_info_error";
	ctrl_value_msg_.data = 0;

	boost::thread thread_sub(boost::bind(&RudderUDPComClass::send_signal_thread, this));
	boost::thread thread_pub(boost::bind(&RudderUDPComClass::publish_info_thread, this));
	boost::thread thread_refresh(boost::bind(&RudderUDPComClass::refresh_info_thread, this));
  }

  /** Destrunctor **/
  ~RudderUDPComClass(){

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
  ros::Publisher pub_angle_; 
  ros::Publisher pub_error_;
  ros::Subscriber sub_ctrl_;
  diagnostic_msgs::DiagnosticStatus msg_error_;
  std_msgs::Int16 ctrl_value_msg_;
  std_msgs::Float32 angle_msg_;
  

  /** Methods **/
  //UDP
  void init_ethernet(void){
	sock_ = socket(AF_INET, SOCK_DGRAM, 0);
	addr_.sin_family = AF_INET;
	addr_.sin_port = htons(50002);
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

	addr_dest.sin_family = AF_INET;
	addr_dest.sin_port = htons(port);
	addr_dest.sin_addr.s_addr = inet_addr(ip);
	
	return sendto(sock_, data, data_length, 0, (struct sockaddr *)&addr_dest, sizeof(sockaddr_in));

  }

  //ROS
  void ctrl_value_sub_callback(const std_msgs::Int16 value){
	char buffer[4];
	char dest_ip[] = RUDDER_IP;
	int send_size;

	memset(buffer, 0, sizeof(buffer));
	memcpy(buffer, &value, sizeof(&value));
	send_size = send_udp(RUDDER_PORT, dest_ip, buffer, sizeof(buffer));
	mtx_.lock();
	ROS_INFO("Sent:%d Byte", send_size);
	mtx_.unlock();
  }

  void refresh_info_thread(){
	char buffer[8];
	
	while(ros::ok()){
	  if(recv_udp(RUDDER_IP, buffer, sizeof(buffer))>0){
		mtx_.lock();
		msg_error_.level = msg_error_.OK;
		msg_error_.message = "";
		memcpy(&(angle_msg_.data), buffer, 4);
		mtx_.unlock();		
	  }else{
		mtx_.lock();
		msg_error_.level = msg_error_.ERROR;
		msg_error_.message = "Missing_Rudder_Info";
		angle_msg_.data = 0;
		mtx_.unlock();
	  }

	}
  }

  void publish_info_thread(){
	ros::Rate loop_rate(10);

	while(ros::ok()){
	  mtx_.lock();
	  pub_angle_.publish(angle_msg_);
	  pub_error_.publish(msg_error_);
	  ROS_INFO("Rudder Angle :%.1f Deg", angle_msg_.data);
	  mtx_.unlock();
	  loop_rate.sleep();
	}
  }

  void send_signal_thread(){
   	while(ros::ok()){
	  ;
	}
  }



};


int main(int argc, char *argv[]){
  ros::init(argc, argv, "NODE_Rudder1");
  RudderUDPComClass rudder_udp;
  ros::spin();
  return 0;
}


	
