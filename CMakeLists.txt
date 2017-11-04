cmake_minimum_required(VERSION 2.8.3)
SET(PROJECTNAME extrinsic_calibration)

project(${PROJECTNAME})

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)

#find_package(Boost QUIET COMPONENTS bind)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  cv_bridge
  image_transport
  geometry_msgs
  message_filters
  tf
  #opencv_candidate
)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

catkin_package(
  INCLUDE_DIRS include/extrinsic_calibration
  CATKIN_DEPENDS roscpp cv_bridge image_transport geometry_msgs
  DEPENDS OpenCV
)

###########
## Build ##
###########

include_directories(
  include/extrinsic_calibration
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

SET(SRC
  src/main.cpp
  src/calibrator.cpp
  src/pose_estimation.cpp
  include/extrinsic_calibration/calibrator.hpp
  include/extrinsic_calibration/pose_estimation.hpp
)

add_executable(${PROJECTNAME} ${SRC})

target_link_libraries(${PROJECTNAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
 )
