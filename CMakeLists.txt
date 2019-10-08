cmake_minimum_required(VERSION 2.8.3)
project(rasberry_perception)

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
  bayes_people_tracker
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
  GetDetectorResults.srv
)

add_message_files(
  FILES
  BoundingBox.msg
  ImageDetections.msg
  SegmentationLabel.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs rasberry_perception)

# ==============================================
# Arm Control Setup
# ==============================================
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

find_library(controlcan NAMES libcontrolcan.so PATHS src/linear_3dof_arm/lib)
include_directories(include include/linear_3dof_arm ${catkin_INCLUDE_DIRS})

add_executable(linear_3dof_arm_canopen_singlearm src/linear_3dof_arm/canopen_singlearm.cpp)
target_link_libraries(linear_3dof_arm_canopen_singlearm ${catkin_LIBRARIES} ${controlcan})
add_dependencies(linear_3dof_arm_canopen_singlearm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ==============================================
# Install
# ==============================================
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB python_scripts_DIR "scripts/*.py")
catkin_install_python(PROGRAMS ${python_scripts_DIR} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
