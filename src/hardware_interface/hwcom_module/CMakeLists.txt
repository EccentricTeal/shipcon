cmake_minimum_required(VERSION 3.5)
project(hwcom_module)

#option(HWCOMLIB_BUILD_SHARED_LIBS "build as a shared library" ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#Build UDPCOM
#if( HWCOMLIB_BUILD_SHARED_LIBS )
add_library(udpcom SHARED
  src/udp_recv_class.cc
  src/udp_send_class.cc
)
#else()
#  add_library(udpcom STATIC
#    src/udp_recv_class.cc
#    src/udp_send_class.cc
#  )
#endif()
target_compile_definitions(udpcom
  PRIVATE "UDPCOM_BUILDING_LIBRARY"
)
ament_target_dependencies(udpcom
  #None
)
target_include_directories(udpcom
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS udpcom
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


ament_package()
