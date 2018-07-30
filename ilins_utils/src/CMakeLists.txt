add_compile_options(-std=c++17)

add_executable(ilins_subscriber_demo ilins_subscriber.cpp)

target_link_libraries(ilins_subscriber_demo 
	${catkin_LIBRARIES}
)