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
    pub_motor_ = pnh_.advertise<shipcon::motor_control>( "motor_command", 1 );
    pub_rudder_ = pnh_.advertise<shipcon::rudder_control>( "rudder_command", 1 );
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
    target_rudder_angle_rad_ = 0.0;
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
      target_motor_rpm_ = MOTOR_POWER_RPM;
      target_propeller_pitch_deg_ = PROPELLER_PITCH_DEG;
    }
    else
    {
      target_motor_rpm_ = 0.0;
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
          target_rudder_angle_rad_ = unitcon::angle::deg2rad( -TARGET_RUDDER_ANGLE_DEG );
          turn_mode_.mode = TurnDirection::TURN_PORT;
        }
        else
        {
          target_rudder_angle_rad_ = unitcon::angle::deg2rad( TARGET_RUDDER_ANGLE_DEG );
        }
      }
      else
      {
        if( current_yaw_rad_ < unitcon::angle::deg2rad( 360.0-THREASHOLD_HEADING_DEG ) )
        {
          target_rudder_angle_rad_ = unitcon::angle::deg2rad( TARGET_RUDDER_ANGLE_DEG );
          turn_mode_.mode = TurnDirection::TURN_STBD;
        }
        else
        {
          target_rudder_angle_rad_ = unitcon::angle::deg2rad( -TARGET_RUDDER_ANGLE_DEG );
        }
      }
    }
    else
    {
      target_rudder_angle_rad_ = 0.0;
    }
  }


  void ZtestNode::publishTopics( void )
  {
    shipcon::motor_control msg_motor;
    shipcon::rudder_control msg_rudder;

    msg_motor.target_rpm = target_motor_rpm_;
    msg_motor.target_pitch_rad = unitcon::angle::deg2rad( target_propeller_pitch_deg_ );
    msg_rudder.target_angle_rad.clear();
    msg_rudder.target_angle_rad.push_back( static_cast<float>( target_rudder_angle_rad_ ) );
    msg_rudder.target_angle_rad.push_back( static_cast<float>( target_rudder_angle_rad_ ) );

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
    if( msg->data == 1 )
    {
      if( isAutomode_ == false )
      {
        initTest();
        isAutomode_ = true;
      }
    }
    else
    {
      isAutomode_ = false;
    }
  }

}

	
