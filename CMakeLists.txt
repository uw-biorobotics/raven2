cmake_minimum_required (VERSION 2.8.3)
project(raven_2_crtk)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  dynamic_reconfigure
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  crtk_msgs
)

find_package(Eigen3 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES raven_automove.msg raven_state.msg 
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs 
)

generate_dynamic_reconfigure_options(
  cfg/Raven2.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp dynamic_reconfigure tf
)

include_directories(
  include
  include/raven
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

set(r2_control_crtk_sources
  src/raven/console_process.cpp
  src/raven/dof.cpp
  src/raven/fwd_cable_coupling.cpp
  src/raven/get_USB_packet.cpp
  src/raven/globals.cpp
  src/raven/grav_comp.cpp
  src/raven/homing.cpp
  src/raven/init.cpp
  src/raven/inv_cable_coupling.cpp
  src/raven/local_io.cpp
  src/raven/log.cpp
  src/raven/mapping.cpp
  src/raven/network_layer.cpp
  src/raven/overdrive_detect.cpp
  src/raven/pid_control.cpp
  src/raven/put_USB_packet.cpp
  src/raven/r2_jacobian.cpp
  src/raven/r2_kinematics.cpp
  src/raven/reconfigure.cpp
  src/raven/rt_process_preempt.cpp
  src/raven/rt_raven.cpp
  src/raven/state_estimate.cpp
  src/raven/state_machine.cpp
  src/raven/t_to_DAC_val.cpp
  src/raven/tools.cpp
  src/raven/crtk_state.cpp
  src/raven/crtk_motion_api.cpp
  src/raven/crtk_motion_planner.cpp
  src/raven/trajectory.cpp
  src/raven/update_atmel_io.cpp
  src/raven/update_device_state.cpp
  src/raven/USB_init.cpp
  src/raven/utils.cpp
)

add_executable(r2_control_crtk ${r2_control_crtk_sources})
add_dependencies(r2_control_crtk ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(r2_control_crtk ${catkin_LIBRARIES})
