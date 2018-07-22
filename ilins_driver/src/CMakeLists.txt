add_compile_options(-std=c++17)

add_library(ilins_serialport serialport.cpp)
target_link_libraries(ilins_serialport
                      ${catkin_LIBRARIES})

add_library(ilins_input input.cpp)
target_link_libraries(ilins_input
                      ilins_serialport
                      ${catkin_LIBRARIES})

add_library(ilins_driver ildriver.cpp)
target_link_libraries(ilins_driver
                      ilins_input
                      ilins_serialport
                      ${catkin_LIBRARIES})

add_executable(ilins_node ilins_node.cpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(ilins_input ilins_serialport ${catkin_EXPORTED_TARGETS})
endif()

#add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ilins_node 
	ilins_driver
  ilins_input
  ilins_serialport
	${catkin_LIBRARIES}
)

 #install(TARGETS my_signal
  #       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
