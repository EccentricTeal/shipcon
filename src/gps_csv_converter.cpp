#include "ros/ros.h"
#include "shipcon/gps_position.h"

#include <stdio.h>
#include <stdlib.h> /*exit()*/
#include <termios.h>
#include <string.h> /*memset()*/
#include <fcntl.h> /*open(), O_RDWR*/
#include <termios.h>
#include <unistd.h> /*read()*/

FILE *csv_fp;

void callback(const shipcon::gps_position::ConstPtr &msg){
  double latitude;
  std::string lat_dir;

  float longitude;
  std::string lon_dir;

  lat_dir = msg->latitude_dir.c_str();
  latitude = msg->latitude_deg + msg->latitude_min /60.0;
  if(lat_dir == "N"){
	;
  }else if(lat_dir == "S"){
	latitude = -latitude;
  }else{
	latitude = -1000.0;
  }

  lon_dir = msg->longitude_dir.c_str();
  longitude = msg->longitude_deg + msg->longitude_min /60.0;
  if(lon_dir == "E"){
	;
  }else if(lon_dir == "W"){
	longitude = -longitude;
  }else{
	longitude = -1000.0;
  }

  fprintf(csv_fp, "%lf,%lf\n", latitude, longitude);
}

int main(int argc, char* argv[]){
  /*Initialize File*/
  if((csv_fp = fopen("gps.csv", "w")) == NULL){
	exit(EXIT_FAILURE);
  }

  /*Initialize ROS*/
  ros::init(argc, argv, "GPS_to_CSV");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("GPS_Position", 1000, callback);

  ros::spin();
  
  fclose(csv_fp);
  return 0;
}
