#include "shipcon/motor.hh"

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "NODE_Motor1" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  shipcon::MotorNode motor( nh, pnh );
  
  motor.run();
  ros::spin();
  
  return 0;
}