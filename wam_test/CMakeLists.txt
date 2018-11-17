cmake_minimum_required(VERSION 2.8.3)
project(wam_test)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  wam_common
)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wam_node
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_srvs wam_common
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
	/usr/include/eigen2/
	/usr/include/xenomai/
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(wam_cart_test src/wam_cartTraj_test.cpp)
target_link_libraries(wam_cart_test ${catkin_LIBRARIES} ${BARRETT_LIBRARIES} gsl)

add_executable(wam_joint_test src/wam_jointTraj_test.cpp)
target_link_libraries(wam_joint_test ${catkin_LIBRARIES} ${BARRETT_LIBRARIES} gsl)

link_directories (BARRETT_LIBRARY_DIRS)


