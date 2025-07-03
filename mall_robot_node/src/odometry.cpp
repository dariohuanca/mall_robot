#include "mall_robot_node/odometry.hpp"

OdometryPublisher::OdometryPublisher()
: Node("odometry"), x_(0.0), y_(0.0), th_(0.0), has_last_time_(false)
{
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  encoder_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "encoder_vel", 10,
    std::bind(&OdometryPublisher::encoder_callback, this, std::placeholders::_1)
  );
}

void OdometryPublisher::encoder_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  rclcpp::Time current_time(msg->header.stamp);

  if (!has_last_time_) {
    last_time_ = current_time;
    has_last_time_ = true;
    return;
  }

  double dt = (current_time - last_time_).seconds();

  double vx = msg->twist.linear.x;
  double vy = msg->twist.linear.y;
  double vth = msg->twist.angular.z;

  double delta_x = (vx * std::cos(th_) - vy * std::sin(th_)) * dt;
  double delta_y = (vx * std::sin(th_) + vy * std::cos(th_)) * dt;
  double delta_th = vth * dt;

  x_ += delta_x;
  y_ += delta_y;
  th_ += delta_th;

  tf2::Quaternion q;
  q.setRPY(0, 0, th_);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  tf_broadcaster_->sendTransform(odom_trans);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.child_frame_id = "base_link";
  odom.twist.twist = msg->twist;

  odom_pub_->publish(odom);
  last_time_ = current_time;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}