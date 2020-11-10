cmake_minimum_required(VERSION 3.5)
project(company_information)

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
find_package(tutorial_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
	"tutorial_interfaces"
	"rclcpp"
	"std_msgs"
)
add_executable(info_pub src/pubinfo.cpp)
ament_target_dependencies(info_pub ${dependencies})

find_package(rosidl_default_generators REQUIRED)
set(msg_file
  "msg/Companyinfo.msg"
  "msg/Stdmsgcolor.msg"
)



###################
# the way to use the variables declared in this file is ${<variable_name>}
# 这里为什么加了依赖 “DEPENDENCIES tutorial_interfaces”，
# 是因为我们在本功能包的"msg/Companyinfo.msg" 中
# 使用了来自 功能包“tutorial_interfaces” 自定义的数据类型，如果不加依赖，就找不到这个数据类型
rosidl_generate_interfaces(${PROJECT_NAME}
 ${msg_file}
 DEPENDENCIES tutorial_interfaces std_msgs
 )


ament_export_dependencies(rosidl_default_runtime)
################




#In order to use the messages generated in the same package
#  we need to use the following CMake code:
rosidl_target_interfaces(info_pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")



install(TARGETS 
info_pub
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
