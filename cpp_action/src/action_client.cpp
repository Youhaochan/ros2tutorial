// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <type_traits>
#include <typeinfo>
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"


/*
创建client的顺序
1. create a client named "fibonacci"
2. write down the callback function 
3. bind the costumed callback function and store them in "send_goal_options"
3. call async_send_goal to send goal to the server (here we can use the timer or only call once)
3. 
*/
namespace act_cli{
class MinimalActionClient : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  //explicit 是为了避免编译器隐士转换
  explicit MinimalActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()) : Node("minimal_action_client", node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci");
    this->declare_parameter<int>("n_param", 100); 
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MinimalActionClient::send_goal, this));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
       using namespace std::placeholders;

    this->timer_->cancel();// 这里取消时钟，避免一直发送goal

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    this->get_parameter("n_param", n);
    goal_msg.order = n;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  
    send_goal_options.goal_response_callback =std::bind(&MinimalActionClient::goal_response_call, this, _1);
    
    send_goal_options.feedback_callback =std::bind(&MinimalActionClient::feedback_call, this, _1, _2); 

    send_goal_options.result_callback = std::bind(&MinimalActionClient::result_call, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  int n = 100;
  bool goal_done_;
 // std::function<void (std::shared_future<typename GoalHandle::SharedPtr>)>
  void goal_response_call(std::shared_future<GoalHandleFibonacci::SharedPtr> goal_handle)
  {
    if (!goal_handle.get()) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
 // std::function<void ( typename ClientGoalHandle<ActionT>::SharedPtr, const std::shared_ptr<const Feedback>)>
  void feedback_call(GoalHandleFibonacci::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Next number in sequence received: %d" ,
      feedback->sequence.back());
  }

  void result_call(const GoalHandleFibonacci::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : result.result->sequence) {
      RCLCPP_INFO(this->get_logger(), "%" PRId32, number);
    }
  }
};  // class MinimalActionClient
}//namespace action_client
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<act_cli::MinimalActionClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}