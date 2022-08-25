#include "shipcon/motor.hh"

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "NODE_Motor1" );
  shipcon::MotorNode motor( "192.168.39.10", 50000 );
  motor.run();
  ros::spin();
  
  return 0;
}