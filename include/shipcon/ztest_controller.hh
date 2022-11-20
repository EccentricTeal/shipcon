#ifndef SHIPCON__ZTEST_CONTROLLER__HH
#define SHIPCON__ZTEST_CONTROLLER__HH

#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "shipcon/motor_control.h"
#include "shipcon/motor_info.h"
#include "shipcon/rudder_control.h"
#include "shipcon/rudder_info.h"
#include "std_msgs/Float32.h"/*Rudder Angle*/
#include "std_msgs/Int16.h"/*Thruster,RC*/
#include "shipcon/gyro.h" /*Gyro*/
#include "shipcon/angle_converter.hh"
#include "shipcon/Jg35fdResetAngle.h"

#include <string>
#include <memory>

namespace shipcon
{
  struct TurnDirection
  {
    static const uint16_t TURN_STBD = 1;
    static const uint16_t TURN_PORT = 0;
    uint16_t mode = TURN_STBD;
  };

  class ZtestNode
  {
    /* Constants */
    private:
      const double MOTOR_POWER_RPM = 1100.0;
      const double PROPELLER_PITCH_DEG = 30.0;
      const double RUDDER_MAX_ANGLE_DEG = 30.0;
      const double TARGET_RUDDER_ANGLE_DEG = 20.0;
      const double THREASHOLD_HEADING_DEG = 20.0;

    /* Private Objects */
    private:
      ros::NodeHandle nh_, pnh_;

      ros::Publisher pub_motor_;
      ros::Publisher pub_rudder_;
      ros::Subscriber sub_gyro_;
      ros::Subscriber sub_radio_control_;
      std::string subname_gyro_;
      std::string subname_radio_control_;
      ros::ServiceClient srv_reset_angle_;
      std::string srvname_reset_angle_;

      TurnDirection turn_mode_;
      double current_yaw_rad_;
      bool isAutomode_;
      double target_motor_rpm_;
      double target_propeller_pitch_deg_;
      double target_rudder_angle_rad_;
    
    /* Constructors */
    public:
      ZtestNode( ros::NodeHandle nh, ros::NodeHandle pnh );
      ~ZtestNode();

    /* Public Methods*/
    public:
      void run( void );

    /* Private Methods */
    private:
      void initPublisher( void );
      void initSubscriber( void );
      void initServiceClient( void );
      void initTest( void );
      bool resetGyro( void );
      void updateMotor( void );
      void updateRudder( void );
      void publishTopics( void );
      void mainLoop( void );

    /* Callback */
    private:
      void subcallback_gyro( shipcon::gyro::ConstPtr msg );
      void subcallback_radio_control(std_msgs::Int16::ConstPtr msg );
  };
}



#endif