cmake_minimum_required(VERSION 3.5)
project(tutorials_client_server)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tutorial_interfaces REQUIRED)


add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces tutorial_interfaces)
add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces tutorial_interfaces)
###############################################
##这里就是一个为了使用自定义数据需要的命令，不管是不是被本功能包调用还是其他功能包调用
## 下面的几句都是必须的
find_package(rosidl_default_generators REQUIRED)
set(srv_file
  "srv/Addthreestr.srv"
)
# the way to use the variables declared in this file is ${<variable_name>}
rosidl_generate_interfaces(${PROJECT_NAME}
 ${srv_file}
 )
###############################################
##############################################
# 
# 下面一行只是因为我们要在同一个功能包里面调用自定义的数据， 
# 调用另外一个包的数据 是不用这句话的
ament_export_dependencies(rosidl_default_runtime)
#In order to use the messages generated in the same package
#  we need to use the following CMake code:
rosidl_target_interfaces(server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
############################################
install(TARGETS 
server
client
DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
