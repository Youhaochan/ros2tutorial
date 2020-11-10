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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/str.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(): Node("minimal_subscriber")
  {
  	// two ways to create the subscriber 
  	// _1 means the first parameter of function &MinimalSubscriber::topic_callback is unknown and needs to be fed in 
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Str>("talker_listener", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    /*subscription_ = this->create_subscription<std_msgs::msg::Str>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, &MinimalSubscriber, _1)); */
  }

/*
The topic_callback function receives the string message data published over the topic, 
and simply writes it to the console using the RCLCPP_INFO macro.
*/
private:
  void topic_callback(const tutorial_interfaces::msg::Str::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->name.c_str());
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Str>::SharedPtr subscription_;
  
};          

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // std::shared_ptr<MinimalSubscriber> sub(new MinimalSubscriber);  ////ok
  std::shared_ptr<MinimalSubscriber> sub = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(sub);// the type of input has to be an shared pointer;
  rclcpp::shutdown();
  return 0;
}
