cmake_minimum_required(VERSION 2.8.3)
project(rasberry_perception)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  ros_numpy
  message_generation
  bayes_tracking
  tf
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

# ==============================================
# Python scripts setup
# ==============================================
catkin_python_setup()

# ==============================================
# Service/Message files setup
# ==============================================
add_service_files(
  FILES
  GetDetectorResults.srv
)

add_message_files(
    FILES
    Detection.msg
    Detections.msg
    RegionOfInterest.msg
    SegmentOfInterest.msg
    ServiceStatus.msg
    TaggedPose.msg
    TaggedPoseArray.msg
    TrackDetection.msg
    TrackerResults.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs rasberry_perception)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS bayes_tracking geometry_msgs roscpp std_msgs tf visualization_msgs
)

# ==============================================
# Build
# ==============================================

include_directories( include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME}_tracking src/${PROJECT_NAME}/tracking/tracking.cpp )

add_dependencies(${PROJECT_NAME}_tracking ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_tracking ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

# ==============================================
# Install
# ==============================================

install(TARGETS ${PROJECT_NAME}_tracking ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
install(DIRECTORY include/${PROJECT_NAME}/tracking/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.hpp" PATTERN ".git" EXCLUDE )
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

file(GLOB python_scripts_DIR "scripts/*.py")
catkin_install_python(PROGRAMS ${python_scripts_DIR} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
