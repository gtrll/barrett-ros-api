cmake_minimum_required(VERSION 2.8.3)
project(wam_node)

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
	include
	include/util
)

file(GLOB LIB_UTIL_SRC "src/util/*.cpp")
add_library(wam_util ${LIB_UTIL_SRC})

add_library(wam_filt src/ft_filter.cpp)

add_executable(wam_node src/wam_node.cpp)
target_link_libraries(wam_node ${catkin_LIBRARIES} ${BARRETT_LIBRARIES} gsl wam_util)

add_executable(ft_filter src/ft_filter.cpp)
target_link_libraries(ft_filter ${catkin_LIBRARIES})

add_executable(ft_offset src/ft_offset.cpp)
target_link_libraries(ft_offset wam_filt ${catkin_LIBRARIES})

link_directories (BARRETT_LIBRARY_DIRS)


