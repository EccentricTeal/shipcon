#ifndef SHIPCON__RADIO_CONTROL__HH
#define SHIPCON__RADIO_CONTROL__HH

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "shipcon/motor_info.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <thread>
#include <string>
#include <mutex>
#include <memory>

namespace shipcon
{
  class RadioControlNode
  {
    /** Constants **/
    private:
    
    /** Member Objects **/
    private:
      //Thread
      std::mutex mtx_;

      //Socket Com
      int sock_;
      struct sockaddr_in addr_;
      std::string ip_addr_;
      int port_;

      //ROS
      ros::NodeHandle nh_, pnh_;
      ros::Publisher pub_rcvalue_motor_;
      ros::Publisher pub_rcvalue_prop_pitch_;
      ros::Publisher pub_rcvalue_rudder_;
      ros::Publisher pub_rcvalue_mode_;
      ros::Publisher pub_rcvalue_gyro_reset_;
      ros::Publisher pub_rcvalue_diaginfo_;
      ros::Subscriber sub_ctrlval_;
      std::string subname_ctrlval_;

      std_msgs::Int16 msg_motor_;
      std_msgs::Int16 msg_prop_pitch_;
      std_msgs::Int16 msg_rudder_;    
      std_msgs::Int16 msg_mode_; /*0:Man, 1:Auto*/
      std_msgs::Int16 msg_gyro_reset_;
      diagnostic_msgs::DiagnosticStatus msg_error_;

      //Thread
      std::unique_ptr<std::thread> threadptr_pub_;
      std::unique_ptr<std::thread> threadptr_update_;

    /** Constructor, Destructor **/
    public:
      RadioControlNode( ros::NodeHandle nh, ros::NodeHandle pnh );
      ~RadioControlNode();

    public:
      void run( void );

    /** Method **/
    private:
      void initEthernet( void );

    /** Thread **/
    private:
      void thread_publish( void );
      void thread_update( void );
  };
}

#endif