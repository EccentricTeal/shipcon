#include "shipcon/ztest_controller.hh"

namespace shipcon
{
  ZtestNode::ZtestNode( ros::NodeHandle nh, ros::NodeHandle pnh ):
  nh_( nh ),
  pnh_( pnh )
  {
    initPublisher();
    initSubscriber();

    current_yaw_rad_ = 0.0;
    isAutomode_ = false;
  }

  
  ZtestNode::~ZtestNode()
  {
    ;
  }


  void ZtestNode::run()
  {
    mainLoop();
  }


  void ZtestNode::initPublisher( void )
  {
    pub_motor_ = pnh_.advertise<std_msgs::Int16>( "motor_command", 1 );
    pub_rudder_ = pnh_.advertise<std_msgs::Int16>( "rudder_command", 1 );
  }


  void ZtestNode::initSubscriber( void )
  {
    pnh_.param<std::string>( "SubnameGyro", subname_gyro_, "gyro" );
    pnh_.param<std::string>( "SubnameRadioControl", subname_radio_control_, "radio_control" );
    sub_gyro_ = nh_.subscribe( subname_gyro_, 1, &ZtestNode::subcallback_gyro, this );
    sub_radio_control_ = nh_.subscribe( subname_radio_control_, 1, &ZtestNode::subcallback_radio_control, this );
  }


  void ZtestNode::initServiceClient( void )
  {
    pnh_.param<std::string>( "SrvnameGyroResetAngle", srvname_reset_angle_, "gyro_reset_angle" );
    srv_reset_angle_ = nh_.serviceClient<shipcon::Jg35fdResetAngle>( srvname_reset_angle_ );
  }


  void ZtestNode::initTest( void )
  {
    target_motor_power_percent_ = MOTOR_POWER_PERCENT;
    target_rudder_angle_percent_ = 0.0;
    turn_mode_.mode = TurnDirection::TURN_STBD;
    resetGyro();
  }


  bool ZtestNode::resetGyro( void )
  {
    shipcon::Jg35fdResetAngle srvdata;
    srvdata.request.yaw_angle_rad = 0.0;
    if( !srv_reset_angle_.call( srvdata ) )
    {
      return false;
    }
    else
    {
      return true;
    }
  }


  void ZtestNode::updateMotor( void )
  {
    if( isAutomode_ )
    {
      target_motor_power_percent_ = MOTOR_POWER_PERCENT;
    }
    else
    {
      target_motor_power_percent_ = 0.0;
    }
  }


  void ZtestNode::updateRudder( void )
  {
    if( isAutomode_ )
    {
      if( turn_mode_.mode == TurnDirection::TURN_STBD )
      {
        if( current_yaw_rad_ > unitcon::angle::deg2rad( THREASHOLD_HEADING_DEG ) )
        {
          target_rudder_angle_percent_ = TARGET_RUDDER_ANGLE_DEG / RUDDER_MAX_ANGLE_DEG;
          turn_mode_.mode = TurnDirection::TURN_PORT;
        }
        else
        {
          target_rudder_angle_percent_ = -TARGET_RUDDER_ANGLE_DEG / RUDDER_MAX_ANGLE_DEG;
        }
      }
      else
      {
        if( current_yaw_rad_ < unitcon::angle::deg2rad( -THREASHOLD_HEADING_DEG ) )
        {
          target_rudder_angle_percent_ = -TARGET_RUDDER_ANGLE_DEG / RUDDER_MAX_ANGLE_DEG;
          turn_mode_.mode = TurnDirection::TURN_STBD;
        }
        else
        {
          target_rudder_angle_percent_ = TARGET_RUDDER_ANGLE_DEG / RUDDER_MAX_ANGLE_DEG;
        }
      }
    }
    else
    {
      target_rudder_angle_percent_ = 0.0;
    }
  }


  void ZtestNode::publishTopics( void )
  {
    std_msgs::Int16 msg_motor;
    std_msgs::Int16 msg_rudder;

    msg_motor.data = static_cast<int16_t>( target_motor_power_percent_ );
    msg_rudder.data = static_cast<int16_t>( target_rudder_angle_percent_ * 100.0 );

    pub_motor_.publish( msg_motor );
    pub_rudder_.publish( msg_rudder );
  }


  void ZtestNode::mainLoop( void )
  {
    ros::Rate loop_rate(10);

    while( ros::ok() )
    {
      ros::spinOnce();

      updateMotor();
      updateRudder();
      publishTopics();

      loop_rate.sleep();
    }
  }


  void ZtestNode::subcallback_gyro( shipcon::gyro::ConstPtr msg )
  {
    current_yaw_rad_ = msg->ang_yaw;
  }


  void ZtestNode::subcallback_radio_control(std_msgs::Int16::ConstPtr msg )
  {
    if( msg->data == 1 && isAutomode_ == false )
    {
      initTest();
      isAutomode_ = true;
    }
    else
    {
      isAutomode_ = false;
    }
  }

}

	
