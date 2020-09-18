const static float K = 0.1;
const static float C = 0.1;
const static float L = 0.001;

#include <functional>
#include <memory>
#include <chrono>
#include <vector>  
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class HumanAssistController : public rclcpp::Node
{
public:
  HumanAssistController()
  : Node("human_assist_controller")
  {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_human_input", 10, std::bind(&HumanAssistController::human_input_callback, this, _1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), std::bind(&HumanAssistController::scan_callback, this, _1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      1ms, std::bind(&HumanAssistController::timer_callback, this));
  }

private:
  void human_input_callback(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    u_h1 = msg->linear.x;
    u_h2 = msg->angular.z;
  }
  void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    r = theta = msg->ranges;
    for (unsigned int i = 0; i < msg->ranges.size(); ++i) {
      if(r[i] > 1.0) r[i] = 10.0;
      theta[i] = msg->angle_min + i * msg->angle_increment;
    }
  }
  void timer_callback()
  {
    if(r.size() == 0) return;

    B = LgpB1 = LgpB2 = 0;

    for (unsigned int i = 0; i < r.size(); ++i) {
      // r_bi = 0.170;  // roomba
      r_bi = 0.105;     // turtlebot3 burger
      dr_bi_dtheta = 0;
  
      B += 1.0/(r[i] -r_bi) + L*(r[i]*r[i] + theta[i]*theta[i]);
      LgpB1 += -(-1.0/((r[i] -r_bi)*(r[i] -r_bi)) + 2.0*L*r[i])*cos(theta[i])
        + (dr_bi_dtheta/((r[i] -r_bi)*(r[i] -r_bi)) + 2.0*L*theta[i])*sin(theta[i])/r[i];
      LgpB2 += (dr_bi_dtheta/((r[i] -r_bi)*(r[i] -r_bi)) + 2.0*L*theta[i]);
    }
    LgpB_sq = LgpB1*LgpB1 + LgpB2*LgpB2;
    I = LgpB1*u_h1 + LgpB2*u_h2;
    J = K*B+C;
    // RCLCPP_INFO(this->get_logger(), "B: '%f',    LgpB1: '%f',    LgpB2: '%f'", B, LgpB1, LgpB2);
    // RCLCPP_INFO(this->get_logger(), "u_h1: '%f',    u_h2: '%f'", u_h1, u_h2);
    // RCLCPP_INFO(this->get_logger(), "u1: '%f',    u2: '%f'", u1, u2);
    // RCLCPP_INFO(this->get_logger(), "I: '%f',    J: '%f'", I, J);
    if(I < J || LgpB_sq == 0){
      u1 = u2 = 0.0;
    } else {
      u1 = - (I - J) /LgpB_sq * LgpB1;
      u2 = - (I - J) /LgpB_sq * LgpB2;
    }
    
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = u1 + u_h1;
    message.angular.z = u2 + u_h2;
    cmd_vel_pub_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::vector<float> r, theta;
  float u1, u2, u_h1, u_h2;
  float r_bi, dr_bi_dtheta;
  float B, LgpB1, LgpB2, LgpB_sq;
  double I, J;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanAssistController>());
  rclcpp::shutdown();
  return 0;
}
