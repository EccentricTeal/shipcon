#include "shipcon/servo.hh"

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "NODE_Servo" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  shipcon::ServoNode servo( nh, pnh );
  
  servo.run();
  ros::spin();
  
  return 0;
}