#include "shipcon/ztest_controller.hh"

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "NODE_ZtestController" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  shipcon::ZtestNode controller( nh, pnh );
  
  controller.run();
  
  return 0;
}