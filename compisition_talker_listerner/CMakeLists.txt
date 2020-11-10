cmake_minimum_required(VERSION 3.5)
project(compisition_talker_listerner)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
##############################  
## 这里面是处理publisher_node.cpp   subscriber_node.cpp 俩个cpp文件的库文件生成
#把publisher_node subscriber_node 转变成一个库文件，最后用composed.cpp调用这个库
add_library(node_compose_talker_listen SHARED
            src/publisher_node.cpp
            src/subscriber_node.cpp)
target_compile_definitions(node_compose_talker_listen 
PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(node_compose_talker_listen rclcpp rclcpp_components std_msgs)
#ament_export_targets(node_compose_talker_listen HAS_LIBRARY_TARGET)
#ament_export_dependencies(some_dependency)


install(TARGETS
  node_compose_talker_listen
 # EXPORT node_compose_talker_listen
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include
)

###############################
#install(
#  DIRECTORY include/
 # DESTINATION include
#)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


########################
# 下面就是单纯的把src/composed.cpp 生成可执行文件 并且安装在lib下 使得ros2 可以找到
#composed.cpp 生成可执行文件 并且链接的库为上面转化的
add_executable(composition_composed src/composed.cpp)
target_link_libraries(composition_composed node_compose_talker_listen)
ament_target_dependencies(composition_composed rclcpp class_loader)
install(TARGETS
  composition_composed
  DESTINATION lib/${PROJECT_NAME})

###############################
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
