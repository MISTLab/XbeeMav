cmake_minimum_required(VERSION 2.8.3)
project(xbee_ros_node)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")
endif()

set (BOOST_INCLUDEDIR "/usr/include")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS
 system)

################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES xbee_ros_node
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


## Specify additional locations of header files
include_directories(
  include ${xbee_ros_node_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(xbee_mav src/Xbee.cpp src/XBeeFrame.cpp src/SerialDevice.cpp  src/CommunicationManager.cpp)
target_link_libraries(xbee_mav ${catkin_LIBRARIES})

#add_executable(config src/main.cpp src/XBeeModule.cpp src/XMLConfigParser.cpp)
#target_link_libraries(config ${catkin_LIBRARIES})

#add_executable(test_controller src/TestController.cpp)
#target_link_libraries(test_controller ${catkin_LIBRARIES})

#add_executable(test_buzz src/TestBuzz.cpp)
#target_link_libraries(test_buzz ${catkin_LIBRARIES})



#############
## Install ##
#############



#############
## Testing ##
#############



