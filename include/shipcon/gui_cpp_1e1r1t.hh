#ifndef SHIPCON__GUI_CPP_1E1R1T__HH
#define SHIPCON__GUI_CPP_1E1R1T__HH

#include "ros/ros.h"
#include "shipcon/MotorInfo.h"
#include "shipcon/Mode.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"

#include <thread>
#include <mutex>
#include <memory>

namespace shipcon
{
  class Dashboard_1E1R1T : public 
  {
    /** Constants **/
    private:

    /** Member Objects **/
    private:
      ros::NodeHandle nh_, pnh_;
      ros::Subscriber sub_motor_;
      ros::Subscirber sub_rudder_;
      ros::Subscriber sub_prop_pitch_;
      ros::Subscriber sub_bow_thruster_;
      ros::Subscriber sub_mode_;
      std::string subname_motor_;
      std::string subname_rudder_;
      std::string subname_prop_pitch_;
      std::string subname_bow_thruster_;
      std::string subname_mode_;

      std::mutex mtx_;
      std::unique_ptr<std::thread> threadptr_subscribe_;

      double motor_rpm_ = 0.0;
      double rudder_deg_ = 0.0;
      double prop_pitch_deg_ = 0.0;
      double bowthruster_rpm_ = 0.0;
      shipcon::Mode mode_ = shipcon::Mode::MODE_STOP;


    /** Constructor, Destuctor **/
    public:
      Dashboard_1E1R1T();
      ~Dashboard_1E1R1T();

    /** Methods **/
    public:
      void run( void );
    
    private:
      void updateDisplay( void );

    /** Subscribe Callback **/
    private:
      void subcallback_motor( shipcon::MotorInfo::ConstPtr msg );
      void subcallback_rudder( std_msgs::Int16::ConstPtr msg );
      void subcallback_prop_pitch( std_msgs::Int16::ConstPtr msg );
      void subcallback_bowthruster( std_msgs::Int16::ConstPtr msg );
      void subcallback_mode( shipcon::Mode::ConstPtr msg );

    /** Thread **/
    private:
      void thread_subscribe( void );

  };
}

#endif