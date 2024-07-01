cmake_minimum_required(VERSION 3.0.2)
project(mpc_steering)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../" "${PROJECT_SOURCE_DIR}/lib" ${CMAKE_MODULE_PATH})
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
include(cpp_standard_selector)
include(cuda_arch_selector)

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
  nav_msgs
  pnc_msgs
  genmsg
  roslib
  rospy
  geometry_msgs
  sensor_msgs
  roscpp
  fmutil
  pcl_ros
  pcl_conversions
  transform_util
  tf
  usbcan_node
  message_generation
  av_map_msgs
  ros_abstraction_layer)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
# add_service_files(DIRECTORY ../golfcar_ppc/srv FILES PPC_Path_Remain.srv)
add_message_files(
  FILES
  Num.msg
)
generate_messages(DEPENDENCIES nav_msgs std_msgs)
# find_library(Cornucopia NAMES libCornucopia.so)
catkin_package(
  INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}
  lib
  lib/generated
  lib/INCLUDE
  lib/SRC
  # ${CMAKE_CURRENT_SOURCE_DIR}
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  pnc_msgs
  fmutil
  usbcan_node
  av_map_msgs
  transform_util
  message_runtime)

# include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_MODULE_PATH} ${EIGEN3_INCLUDE_DIR})

# add_executable(mpc_kinematic_bicycle mpc_kinematic_bicycle.cpp)
# target_link_libraries(mpc_kinematic_bicycle)
# add_dependencies(mpc_kinematic_bicycle ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(mpc_steering ${catkin_LIBRARIES})

add_library(mpc_steering mpc_steering.cpp
mpc_tracking_err_calc.cpp
lib/generated/acado_auxiliary_functions.c
lib/generated/acado_integrator.c
lib/generated/acado_qpoases_interface.cpp
lib/generated/acado_solver.c
lib/SRC/Bounds.cpp
lib/SRC/Constraints.cpp
lib/SRC/CyclingManager.cpp
lib/SRC/Indexlist.cpp
lib/SRC/MessageHandling.cpp
lib/SRC/QProblem.cpp
lib/SRC/QProblemB.cpp
lib/SRC/SubjectTo.cpp
lib/SRC/Utils.cpp)
add_dependencies(mpc_steering ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mpc_steering ${catkin_LIBRARIES})
target_include_directories(mpc_steering PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(
  TARGETS mpc_steering
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
