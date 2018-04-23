cmake_minimum_required(VERSION 2.8.3)
project(map_turtle)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS}
  #LIBRARIES rel_ekf
  CATKIN_DEPENDS roscpp std_msgs image_transport sensor_msgs cv_bridge
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#add_executable(example src/example.cpp)
#add_dependencies(example ${catkin_EXPORTED_TARGETS})
#target_link_libraries(example ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(track 
    src/feature_tracker/feature_tracker.cpp
    src/feature_tracker/estimator.cpp
    src/feature_tracker/node.cpp)
add_dependencies(track ${catkin_EXPORTED_TARGETS})
target_link_libraries(track ${OpenCV_LIBS} ${catkin_LIBRARIES})
