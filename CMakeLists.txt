cmake_minimum_required(VERSION 2.8.3)
project(shipcon)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rqt_gui
  rqt_gui_py
  urdf
  std_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   gyro.msg
   gps_position.msg
   gps_local_position.msg
   gps_navinfo.msg
   gps_time.msg
   actuators.msg
   error.msg
   mode.msg
   motor_info.msg
)
 generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES shipcon
  CATKIN_DEPENDS roscpp rospy rqt_gui rqt_gui_py std_msgs message_runtime
#  DEPENDS system_lib
)




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(motor
  src/class/motor.cc
  src/node/motor_node.cc
)
target_link_libraries(motor ${catkin_LIBRARIES})
add_dependencies(motor shipcon_generate_messages_cpp)


add_executable(servo
  src/class/servo.cc
  src/node/servo_node.cc
)
target_link_libraries(servo ${catkin_LIBRARIES})
add_dependencies(servo shipcon_generate_messages_cpp)


add_executable(rudder
  src/class/rudder.cc
  src/node/rudder_node.cc
)
target_link_libraries(rudder ${catkin_LIBRARIES})
add_dependencies(rudder shipcon_generate_messages_cpp)


add_executable(radio_control
  src/class/radio_control.cc
  src/node/radio_control_node.cc
)
target_link_libraries(radio_control ${catkin_LIBRARIES})
add_dependencies(radio_control shipcon_generate_messages_cpp)


add_executable(gyro
  src/node/gyro_node.cpp
)
target_link_libraries(gyro ${catkin_LIBRARIES})
add_dependencies(gyro shipcon_generate_messages_cpp)


add_executable(gps_csv_converter src/gps_csv_converter.cpp)
target_link_libraries(gps_csv_converter ${catkin_LIBRARIES})
add_dependencies(gps_csv_converter shipcon_generate_messages_cpp)


add_executable(main_control src/main_control.cpp)
target_link_libraries(main_control ${catkin_LIBRARIES})
add_dependencies(main_control shipcon_generate_messages_cpp)


add_executable(logger src/logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES})
add_dependencies(logger shipcon_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/varables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(
  PROGRAMS scripts/gps_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )



## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   plugin.xml
   # myfile2
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_shipcon.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#add_executable(gyro_pub scripts/gyro_pub.py)
#target_link_libraries(gyro_pub ${catkin_LIBRARIES})



