#include "shipcon/gui_cpp_1e1r1t.hh"

namespace shipcon
{da
  Dashboard_1E1R1T::Dashboard_1E1R1T()
  {
    pnh_.Param( "subname_motor", subname_motor_, "Motor" );
    pnh_.Param( "subname_rudder", subname_rudder_, "Rudder" );
    pnh_.Param( "subname_prop_pitch", subname_prop_pitch_, "PropPtich" );
    pnh_.Param( "subname_bow_thruster", subname_bow_thruster_, "BowThruster" );
    pnh_.Param( "subname_mode", subname_mode_, "Mode" );

    nh_.subscribe<shipcon::MotorInfo>( subname_motor_, 1, &Dashboard_1E1R1T::subcallback_motor, this );
    nh_.subscribe<std_msgs::Int16>( subname_rudder_, 1, &Dashboard_1E1R1T::subcallback_rudder, this );
    nh_.subscribe<std_msgs::Int16>( subname_prop_pitch_, 1, &Dashboard_1E1R1T::subcallback_prop_pitch, this );
    nh_.subscribe<std_msgs::Int16>( subname_bow_thruster_, 1, &Dashboard_1E1R1T::subcallback_bowthruster, this );
    nh_.subscribe<shipcon::Mode>( subname_mode_, 1, &Dashboard_1E1R1T::subcallback_mode, this );
  }


  Dashboard_1E1R1T::~Dashboard_1E1R1T()
  {
    if( threadptr_subscribe_ -> joinable() ){ threadptr_subscribe_->join(); }
  }


  void Dashboard_1E1R1T::run( void )
  {
    threadptr_subscribe_ = std::make_unique<std::thread>( &Dashboard_1E1R1T::thread_subscribe, this );
  }


  void Dashboard_1E1R1T::updateDisplay( void )
  {
    
  }


  void Dashboard_1E1R1T::subcallback_motor( shipcon::MotorInfo::ConstPtr msg )
  {
    std::lock_guard<std::mutex> lock( mtx_ );
    motor_rpm_ = ;
  }


  void Dashboard_1E1R1T::subcallback_rudder( std_msgs::Int16::ConstPtr msg )
  {
    std::lock_guard<std::mutex> lock( mtx_ );
    rudder_deg_ = ;
  }


  void Dashboard_1E1R1T::subcallback_prop_pitch( std_msgs::Int16::ConstPtr msg )
  {
    std::lock_guard<std::mutex> lock( mtx_ );
    prop_pitch_deg_ = ;
  }


  void Dashboard_1E1R1T::subcallback_bowthruster( std_msgs::Int16::ConstPtr msg )
  {
    std::lock_guard<std::mutex> lock( mtx_ );
    bowthruster_rpm_ = ;
  }


  void Dashboard_1E1R1T::subcallback_mode( shipcon::Mode::ConstPtr msg )
  {
    std::lock_guard<std::mutex> lock( mtx_ );
    mode = ;
  }


  void Dashboard_1E1R1T::thread_subscribe( void )
  {
    ros::Rate rate( 10 );

    while( ros::ok() )
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
}