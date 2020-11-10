cmake_minimum_required(VERSION 3.5)
project(tutorial_interfaces)

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

find_package(rosidl_default_generators REQUIRED)
# the first way to generate this interfaces
#rosidl_generate_interfaces(${PROJECT_NAME}
 # "msg/Str.msg"
 # "srv/Addthreestr.srv"
 #)
# the second way to do that which is done by declaring an variable
set(msg_file
  "msg/Str.msg"
  "msg/Company.msg"
)
set(srv_file
  "srv/Addthreestr.srv"
)
set(action_file
   "action/Fibonacci.action"
)

# the way to use the variables declared in this file is ${<variable_name>}
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_file}
 ${srv_file}
 ${action_file}
 )
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
