//standard file
#include <memory>

//
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "tutorial_interfaces/srv/addthreestr.hpp"
#include "tutorials_client_server/srv/addthreestr.hpp"
void add(const std::shared_ptr<tutorials_client_server::srv::Addthreestr::Request> request,
          std::shared_ptr<tutorials_client_server::srv::Addthreestr::Response>  response)
{
  for(size_t i=0; i<request->name.size();i++)
  {
    response->sum =response->sum + request->name[i];
    std::cout<<response->sum<<std::endl;
  }
  // 下面的注释 使用在
  // srv文件的内容为 string name1 string name2 string name3
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n name1: %s name2: %s name3: %s " 
  // 	,request->name1.c_str(), request->name2.c_str(),request->name3.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %s", response->sum.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  /*
  //这个option 好像不加也是可以跑的
  */
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions();
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server",options);

  rclcpp::Service<tutorials_client_server::srv::Addthreestr>::SharedPtr service =
    node->create_service<tutorials_client_server::srv::Addthreestr>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add your name.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
