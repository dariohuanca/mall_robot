#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher();  // Constructor

private:
  // Callback for encoder velocity messages
  void encoder_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // ROS 2 publishers and subscribers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr encoder_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Odometry state
  double x_, y_, th_;
  rclcpp::Time last_time_;
  bool has_last_time_;
};

#endif  // ODOMETRY_HPP_