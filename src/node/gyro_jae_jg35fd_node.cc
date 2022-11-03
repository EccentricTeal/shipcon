#include "shipcon/gyro_jae_jg35fd.hh"
#include "ros/ros.h"

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "NODE_Gyro_JAEJG35FD" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  shipcon::GyroJg35fd gyro( nh, pnh );
  ros::spin();
  
  return 0;
}