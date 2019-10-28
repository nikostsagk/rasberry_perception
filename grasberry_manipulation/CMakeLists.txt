cmake_minimum_required(VERSION 2.8.3)
project(grasberry_manipulation)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  realsense2_camera
  ros_numpy
  message_generation
  dynamic_reconfigure
)

# ==============================================
# Python scripts setup
# ==============================================
catkin_python_setup()

# ==============================================
# Service/Message files setup
# ==============================================
add_service_files(
  FILES
  OpenCloseGripper.srv
  ArmControllerService.srv
)

#add_message_files(
#  FILES
#)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

# ==============================================
# Arm Control Setup
# ==============================================
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

find_library(controlcan NAMES libcontrolcan.so PATHS src/arm_3dof/lib)
include_directories(include include/arm_3dof ${catkin_INCLUDE_DIRS})

add_executable(linear_arm_3dof_canopen src/arm_3dof/canopen_singlearm.cpp)
target_link_libraries(linear_arm_3dof_canopen ${catkin_LIBRARIES} ${controlcan})
add_dependencies(linear_arm_3dof_canopen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scara_arm_3dof_canopen src/arm_3dof/canopen_scara.cpp)
target_link_libraries(scara_arm_3dof_canopen ${catkin_LIBRARIES} ${controlcan})
add_dependencies(scara_arm_3dof_canopen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ==============================================
# Install
# ==============================================
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB python_scripts_DIR "scripts/*.py")
catkin_install_python(PROGRAMS ${python_scripts_DIR} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
