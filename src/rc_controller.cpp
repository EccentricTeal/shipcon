  #include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "diagnostic_msgs/DiagnosticStatus.h"

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

const std::string RC_IP = "192.168.39.100";
const int addr_src_len = sizeof(sockaddr_in);

int main(int argc, char* argv[]){
  /*Declaration of Local Variables*/
  int sock;
  struct sockaddr_in addr;
  struct sockaddr_in addr_src;
  std::string recv_ip_addr;
  
  char in_buf[10];
  std_msgs::Int16 msg_thrust1;
  std_msgs::Int16 msg_rudder1;
  diagnostic_msgs::DiagnosticStatus msg_error;
  std_msgs::Int16 msg_mode; /*0:Man, 1:Auto*/
  std_msgs::Int16 msg_gyrosw;

  /*initialize Local Variables*/
  msg_error.name = "RC_Connection_Status";

  /*Initialize Ethernet*/
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(50000);
  addr.sin_addr.s_addr = INADDR_ANY;
  bind(sock, (struct sockaddr *)&addr, sizeof(addr));  

  /*Initialize ROS*/
  ros::init(argc, argv, "NODE_Rc");
  ros::NodeHandle n;
  ros::Publisher pub_motor1 = n.advertise<std_msgs::Int16>("rc_motor1", 1);
  ros::Publisher pub_rudder1 = n.advertise<std_msgs::Int16>("rc_rudder1", 1);
  ros::Publisher pub_mode = n.advertise<std_msgs::Int16>("rc_autosw", 1);
  ros::Publisher pub_gyrosw = n.advertise<std_msgs::Int16>("rc_gyrosw", 1);
  ros::Publisher pub_error = n.advertise<diagnostic_msgs::DiagnosticStatus>("rc_error", 1);
  ros::Rate loop_rate(10);


  /*Main*/
  while(ros::ok()){
	//Initialize Variables
	memset(in_buf, 0, sizeof(in_buf));
	memset(&addr_src, '\0', sizeof(addr_src));
	msg_thrust1.data = 0;
	msg_rudder1.data = 0;
	msg_error.level = msg_error.OK;
	msg_mode.data = 0;


	

	//Receiving data from RC via ethernet
	if(recvfrom(sock, in_buf, sizeof(in_buf), 0, (struct sockaddr *)&addr_src, (socklen_t *)&addr_src_len)==-1){
	  msg_error.level = msg_error.ERROR;
	  //msg_error.message = "ERRNO:" + to_string(errno);
	}
	recv_ip_addr = inet_ntoa(addr_src.sin_addr);
	if(recv_ip_addr == RC_IP){

	  //Copy values
	  memcpy(&(msg_thrust1.data),&in_buf[0], 2);
	  memcpy(&(msg_rudder1.data),&in_buf[2], 2);
	  msg_mode.data = in_buf[4];
	  msg_gyrosw.data = in_buf[6];
	  

	  //Publish
	  ROS_INFO("Throttle:%d, Rudder:%d, AutoFlag:%d, GyroSW:%d", msg_thrust1.data, msg_rudder1.data, msg_mode.data, msg_gyrosw.data);
	  pub_motor1.publish(msg_thrust1);
	  pub_rudder1.publish(msg_rudder1);
	  pub_error.publish(msg_error);
	  pub_mode.publish(msg_mode);
	  pub_gyrosw.publish(msg_gyrosw);

	  /*loop_rate.sleep();*/
	}


  }

  return 0;
}
