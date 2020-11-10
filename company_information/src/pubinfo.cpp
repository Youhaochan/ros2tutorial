#include <chrono> // timer
#include <functional> // 
#include <memory>// smart pointer 
#include <string>

#include "rclcpp/rclcpp.hpp" //rclcpp/rclcpp.hpp include which allows you to use the most common pieces of the ROS 2 system
#include "tutorial_interfaces/msg/company.hpp"
#include "tutorial_interfaces/msg/str.hpp"
#include "company_information/msg/companyinfo.hpp"
#include "company_information/msg/stdmsgcolor.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class com_publisher : public rclcpp::Node //         
{
public:
//用初始化列表方式调用Node节点的构造函数
  com_publisher(): Node("com_publisher"), count_(0), times(0)
  {
    //define a parameter called "com_info"
    this->declare_parameter<std::string>("com_info","Haochan.You"); 
  	publisher_ = this->create_publisher<company_information::msg::Companyinfo>("company_info", 10);
    publisher2_ = this->create_publisher<company_information::msg::Stdmsgcolor>("color_info", 10);
  	// while(times--)
  	// while__callback( times);
    // the line below is another way to call the callback function
  //  which uses the timer to call the call_back function repeatedly
    timer_ = this->create_wall_timer(500ms, std::bind(&com_publisher::timer_callback, this));//every 500ms, we call the timer_callback function
    timer2_ = this->create_wall_timer(1s, std::bind(&com_publisher::timer_callback2, this));//every 500ms, we call the timer_callback function
  }

private:
  void timer_callback()
  {
    auto message_vector = company_information::msg::Companyinfo();
    auto message = tutorial_interfaces::msg::Company();
    {
      message.name_com = "Dorabot";
      message.name_bos = "Xiaobai Deng";
      message.address = "Chigang Station";
      message.num_girl = 30;
      message.num_boy = 70;
      message.num_sum = message.num_girl+message.num_boy;
      this->get_parameter("com_info", message.name_bos);
      message_vector.companys.push_back(message);
    }

    
    
    // std::cout<<"parameters are: "<<msg.name_com<<"  "<<msg.name_bos<<std::endl;
    for(auto data:message_vector.companys)
    {
      RCLCPP_INFO(this->get_logger(), "Company name: '%s'\n Owner name: '%s' ", data.name_com.c_str(),data.name_bos.c_str());
    }        
    publisher_->publish(message_vector);  
  }
   void timer_callback2()
  {
  
    auto message_color = std::make_shared<company_information::msg::Stdmsgcolor>();
    {
      message_color->color.r = 15.8;
      message_color->color.b = 35.8;
      message_color->color.g = 25.8;
      message_color->color.a = 75.8;

    }
  
    RCLCPP_INFO(this->get_logger(), " the times '%d' color is: rbga'%f' '%f' '%f' '%f' \n ", times++,
      message_color->color.r, message_color->color.b, message_color->color.g, message_color->color.a);
    publisher2_->publish(*message_color);
  }


  rclcpp::TimerBase::SharedPtr timer_,timer2_;
  rclcpp::Publisher<company_information::msg::Companyinfo>::SharedPtr publisher_;
  rclcpp::Publisher<company_information::msg::Stdmsgcolor>::SharedPtr publisher2_;
  tutorial_interfaces::msg::Str message = tutorial_interfaces::msg::Str();
  // message.name_com = "Huawei";
  // message.name_bos = "Chengdeng Yu";
  // message.address = "Shenzhen station";
  // message.num_girl = 300;
  // message.num_boy = 7000;
  // message.num_sum = message.num_girl+message.num_boy;
      
  
  size_t count_;
  std::string ss;
  size_t times;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<com_publisher> pub = std::make_shared<com_publisher>();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  return 0;
}
