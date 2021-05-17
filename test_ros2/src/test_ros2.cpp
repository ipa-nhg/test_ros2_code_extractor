#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

void print_usage()
{
  printf("Usage for test_ros2 app:\n");
  printf("..... \n");
  printf("..... \n");
  printf("..... \n");
}

  class test_ros2 : public rclcpp::Node {
    public:
      test_ros2() : Node("test_ros2") {
        string_pub_ = this->create_publisher<std_msgs::msg::String>("string_pub",10);
        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_pub",10);
        bool_sub_ = this->create_subscription<std_msgs::msg::Bool>("bool_sub", 10, std::bind(&test_ros2::bool_sub_callback, this, _1));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_sub", 10, std::bind(&test_ros2::image_sub_callback, this, _1));

        timer_ = this->create_wall_timer(500ms, std::bind(&test_ros2::timer_callback, this));

      }

    private:
      // Subscriber callback
      void bool_sub_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "bool_sub topic got a message");
      }

      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_ ;
      // Subscriber callback
      void image_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "image_sub topic got a message");
      }

      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_ ;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
      rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
      // Timer Callback
      void timer_callback(){
        auto string_pub_msg = std_msgs::msg::String();
        //string_pub_msg = ...
        string_pub_->publish(string_pub_msg);
        RCLCPP_INFO(this->get_logger(), "string_pub publisher active");
        auto battery_pub_msg = sensor_msgs::msg::BatteryState();
        //battery_pub_msg = ...
        battery_pub_->publish(battery_pub_msg);
        RCLCPP_INFO(this->get_logger(), "battery_pub publisher active");
      }
      rclcpp::TimerBase::SharedPtr timer_;

      
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<test_ros2>());
  rclcpp::shutdown();
  return 0;
}
