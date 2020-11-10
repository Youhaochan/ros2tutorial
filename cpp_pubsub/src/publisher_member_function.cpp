// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono> // timer
#include <functional> // 
#include <memory>// smart pointer 
#include <string>
// #include "compisition_talker_listerner/publisher_node.hpp"

#include "rclcpp/rclcpp.hpp" //rclcpp/rclcpp.hpp include which allows you to use the most common pieces of the ROS 2 system
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/str.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node //         
{
public:
//用初始化列表方式调用Node节点的构造函数
  MinimalPublisher(): Node("minimal_publisher"), count_(0), times(5)
  {
  	publisher_ = this->create_publisher<tutorial_interfaces::msg::Str>("talker_listener", 10);
  	while(times--)
  	while__callback( times);
    // the line below is another way to call the callback function
  //  which uses the timer to call the call_back function repeatedly
   //std::bind(&MinimalPublisher::timer_callback, this) bind之后的函数扔进timer里面， 里面会自动去调用这个函数
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));//every 500ms, we call the timer_callback function
  }

private:
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::Str();
    message.name = "Hello, haochan! " + std::to_string(count_++);
    ss = "haochan";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.name.c_str());
    publisher_->publish(message);
  }
  void while__callback(size_t times)
  {
  	auto message = tutorial_interfaces::msg::Str();
  	message.name = "Hello, count down";
  	message.name = message.name +"  "+ std::to_string(times);
  	RCLCPP_INFO(this->get_logger(),"%s",message.name.c_str());
  	publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Str>::SharedPtr publisher_;
  size_t count_;
  std::string ss;
  size_t times;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MinimalPublisher> pub = std::make_shared<MinimalPublisher>();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  return 0;
}
