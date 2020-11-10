#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <typeinfo>
#include "tutorial_interfaces/srv/addthreestr.hpp"
#include "tutorials_client_server/srv/addthreestr.hpp"
#include "tutorial_interfaces/msg/str.hpp"
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y Z");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<tutorials_client_server::srv::Addthreestr>::SharedPtr client =
    node->create_client<tutorials_client_server::srv::Addthreestr>("add_two_ints");

  auto request = std::make_shared<tutorials_client_server::srv::Addthreestr::Request>();
  std::string ss = argv[1];
  request->name.push_back(ss);
  ss = argv[2];
  request->name.push_back(ss);
  ss = argv[3];
  request->name.push_back(ss);


  // request->name2 = argv[2];
  // request->name3 = argv[3];
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s", result.get()->sum.c_str());
    std::cout<<" print the type of result\n"<<typeid(result).name()<<std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
