cmake_minimum_required(VERSION 2.8.3)
project(ilins_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    roslib
    sensor_msgs
    tf
    diagnostic_updater 
    ilins_msgs
    
)

find_package(catkin REQUIRED COMPONENTS 
	${${PROJECT_NAME}_CATKIN_DEPS} 
	rospy 
	std_msgs
  genmsg
  message_generation
)

find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)


include_directories(
  include
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
)


catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    CATKIN_DEPENDS message_runtime std_msgs
    )

add_subdirectory(src)


 #install(TARGETS my_signal
  #       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)