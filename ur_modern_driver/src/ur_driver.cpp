#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ur_msgs/msg/i_o_states.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <ur_msgs/srv/set_i_o.hpp>
#include <ur_msgs/srv/set_payload.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

void print_usage()
{
  printf("Usage for ur_driver app:\n");
  printf("..... \n");
  printf("..... \n");
  printf("..... \n");
}

  class ur_driver : public rclcpp::Node {
    public:
      ur_driver() : Node("ur_driver") {
        joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        wrench_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench",10);
        tool_velocity_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("tool_velocity",10);
        ur_driverio_states_ = this->create_publisher<ur_msgs::msg::IOStates>("ur_driver/io_states",10);
        ur_driverjoint_speed_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("ur_driver/joint_speed", 10, std::bind(&ur_driver::ur_driverjoint_speed_callback, this, _1));
        ur_driverURScript_ = this->create_subscription<std_msgs::msg::String>("ur_driver/URScript", 10, std::bind(&ur_driver::ur_driverURScript_callback, this, _1));
        ur_driverset_io_ = this->create_service<ur_msgs::srv::SetIO>("ur_driver/set_io", std::bind(&ur_driver::ur_driverset_io_handle, this, _1, _2, _3));
        ur_driverset_payload_ = this->create_service<ur_msgs::srv::SetPayload>("ur_driver/set_payload", std::bind(&ur_driver::ur_driverset_payload_handle, this, _1, _2, _3));

        timer_ = this->create_wall_timer(500ms, std::bind(&ur_driver::timer_callback, this));

      }

    private:
      // Subscriber callback
      void ur_driverjoint_speed_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "ur_driver/joint_speed topic got a message");
      }

      rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr ur_driverjoint_speed_ ;
      // Subscriber callback
      void ur_driverURScript_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "ur_driver/URScript topic got a message");
      }

      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ur_driverURScript_ ;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
      rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_;
      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tool_velocity_;
      rclcpp::Publisher<ur_msgs::msg::IOStates>::SharedPtr ur_driverio_states_;
      // Timer Callback
      void timer_callback(){
        auto joint_states_msg = sensor_msgs::msg::JointState();
        //joint_states_msg = ...
        joint_states_->publish(joint_states_msg);
        RCLCPP_INFO(this->get_logger(), "joint_states publisher active");
        auto wrench_msg = geometry_msgs::msg::WrenchStamped();
        //wrench_msg = ...
        wrench_->publish(wrench_msg);
        RCLCPP_INFO(this->get_logger(), "wrench publisher active");
        auto tool_velocity_msg = geometry_msgs::msg::TwistStamped();
        //tool_velocity_msg = ...
        tool_velocity_->publish(tool_velocity_msg);
        RCLCPP_INFO(this->get_logger(), "tool_velocity publisher active");
        auto ur_driverio_states_msg = ur_msgs::msg::IOStates();
        //ur_driverio_states_msg = ...
        ur_driverio_states_->publish(ur_driverio_states_msg);
        RCLCPP_INFO(this->get_logger(), "ur_driver/io_states publisher active");
      }
      rclcpp::TimerBase::SharedPtr timer_;

      
      // Service Handler
      void ur_driverset_io_handle( const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ur_msgs::srv::SetIO::Request> request_,
        const std::shared_ptr<ur_msgs::srv::SetIO::Response> response_){
        (void)request_header;
        RCLCPP_INFO( this->get_logger(), "trigger service '%s'","ur_driver/set_io");
      }
      rclcpp::Service<ur_msgs::srv::SetIO>::SharedPtr ur_driverset_io_;
      // Service Handler
      void ur_driverset_payload_handle( const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ur_msgs::srv::SetPayload::Request> request_,
        const std::shared_ptr<ur_msgs::srv::SetPayload::Response> response_){
        (void)request_header;
        RCLCPP_INFO( this->get_logger(), "trigger service '%s'","ur_driver/set_payload");
      }
      rclcpp::Service<ur_msgs::srv::SetPayload>::SharedPtr ur_driverset_payload_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ur_driver>());
  rclcpp::shutdown();
  return 0;
}
