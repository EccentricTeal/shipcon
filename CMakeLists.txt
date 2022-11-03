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
   motor_control.msg
   bowthruster_info.msg
   bowthruster_control.msg
   rudder_control.msg
   rudder_info.msg
   rudder_vectwin_mode.msg
)

add_service_files(FILES
  Jg35fdCalibrateBiasDrift.srv
  Jg35fdControlCalculate.srv
  Jg35fdControlOutput.srv
  Jg35fdResetAngle.srv
  Jg35fdSetAnalogMode.srv
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


add_executable(bowthruster
  src/class/bowthruster.cc
  src/node/bowthruster_node.cc
)
target_link_libraries(bowthruster ${catkin_LIBRARIES})
add_dependencies(bowthruster shipcon_generate_messages_cpp)


add_executable(single_rudder
  src/class/single_rudder.cc
  src/node/single_rudder_node.cc
)
target_link_libraries(single_rudder ${catkin_LIBRARIES})
add_dependencies(single_rudder shipcon_generate_messages_cpp)


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


add_executable(gyro_jae_jg35fd
  src/class/gyro_jae_jg35fd.cc
  src/node/gyro_jae_jg35fd_node.cc
  src/class/angle_converter.cc
  src/class/serialcom.cc
)
target_link_libraries(gyro_jae_jg35fd
  ${catkin_LIBRARIES}
)
add_dependencies(gyro_jae_jg35fd
  shipcon_generate_messages_cpp
)


add_executable(gps_csv_converter src/gps_csv_converter.cpp)
target_link_libraries(gps_csv_converter ${catkin_LIBRARIES})
add_dependencies(gps_csv_converter shipcon_generate_messages_cpp)


add_executable(ztest_controller
  src/class/ztest_controller.cc
  src/node/ztest_controller_node.cc
  src/class/angle_converter.cc
)
target_link_libraries(ztest_controller
  ${catkin_LIBRARIES}
)
add_dependencies(ztest_controller
  shipcon_generate_messages_cpp
)


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



