#ifndef HUMAN_ASSIST_CONTROLLER_HPP_
#define HUMAN_ASSIST_CONTROLLER_HPP_

#include <functional>
#include <memory>
#include <chrono>
#include <vector>  
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class HumanAssistController : public rclcpp::Node
{
public:
  HumanAssistController();
  ~HumanAssistController();

private:
  void human_input_callback(geometry_msgs::msg::Twist::SharedPtr msg);
  void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timer_callback();
  void param_update_timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr param_update_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::vector<float> r, theta;
  float u1, u2, u_h1, u_h2;
  float r_bi, dr_bi_dtheta;
  float B, LgpB1, LgpB2, LgpB_sq;
  double I, J;
  float K, C, L;
};

#endif // HUMAN_ASSIST_CONTROLLER_HPP_
