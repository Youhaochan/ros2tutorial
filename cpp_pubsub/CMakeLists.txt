cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(include)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorials REQUIRED)
find_package(compisition_talker_listerner REQUIRED IMPORTED)
find_package(tutorial_interfaces REQUIRED)
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs tutorials tutorial_interfaces compisition_talker_listerner)
add_executable(listener src/subscriber_member_function.cpp)					
ament_target_dependencies(listener rclcpp std_msgs tutorials tutorial_interfaces
compisition_talker_listerner) 

# save the executable file talker to directory lib/cpp_pubsub
install(TARGETS
	talker
	listener
	DESTINATION lib/${PROJECT_NAME}
)
install(
DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)
# save the executable file listener to directory lib
#install(TARGETS
#	
#	listener
#	DESTINATION lib
#)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
