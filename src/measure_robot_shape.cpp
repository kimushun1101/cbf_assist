#include "cbf_assist/human_assist_controller.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

HumanAssistController::HumanAssistController()
: Node("human_assist_controller")
{
  this->declare_parameter<std::float_t>("ctrl_param_K", 0.1);
  this->declare_parameter<std::float_t>("ctrl_param_C", 0.1);
  this->declare_parameter<std::float_t>("ctrl_param_L", 0.001);
  this->get_parameter("ctrl_param_K", K);
  this->get_parameter("ctrl_param_C", C);
  this->get_parameter("ctrl_param_L", L);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "human_vel", 10, std::bind(&HumanAssistController::human_input_callback, this, _1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), std::bind(&HumanAssistController::scan_callback, this, _1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_ = this->create_wall_timer(
    1ms, std::bind(&HumanAssistController::timer_callback, this));
  param_update_timer_ = this->create_wall_timer(
    1000ms, std::bind(&HumanAssistController::param_update_timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Human assist control node has been initialised");
}

HumanAssistController::~HumanAssistController()
{
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = 0.0;
  message.angular.z = 0.0;
  cmd_vel_pub_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Human assist control node has been terminated");
}

void HumanAssistController::human_input_callback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  u_h1 = msg->linear.x;
  u_h2 = msg->angular.z;
}

void HumanAssistController::scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  r = theta = msg->ranges;
  for (unsigned int i = 0; i < msg->ranges.size(); ++i) {
    if(r[i] > msg->range_max) r[i] = msg->range_max;
    theta[i] = msg->angle_min + i * msg->angle_increment;
  }
}

void HumanAssistController::timer_callback()
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

void HumanAssistController::param_update_timer_callback()
{
  this->get_parameter("ctrl_param_K", K);
  this->get_parameter("ctrl_param_C", C);
  this->get_parameter("ctrl_param_L", L);
  // RCLCPP_INFO(this->get_logger(), "K: '%f',    C: '%f',    L: '%f'", K, C, L);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanAssistController>());
  rclcpp::shutdown();
  return 0;
}
