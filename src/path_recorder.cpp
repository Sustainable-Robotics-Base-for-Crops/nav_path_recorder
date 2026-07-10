// Copyright 2026 SABI AGRI

#include "nav_path_recorder/path_recorder.hpp"
#include "nav_util/geometry_conversion.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace nav_path_recorder
{
PathRecorder::PathRecorder(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("path_recorder", options)
{
  this->declare_parameter("path.filtering_degree", path_filtering_degree_);
  this->declare_parameter("vehicle_id", path_vehicle_name_);
}

LNI::CallbackReturn PathRecorder::on_configure(const rclcpp_lifecycle::State&)
{
  this->get_parameter("path.filtering_degree", path_filtering_degree_);
  this->get_parameter("vehicle_id", path_vehicle_name_);

  if (path_vehicle_name_.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Need a vehicle_id to save path");
    return LNI::CallbackReturn::FAILURE;
  }

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path_performed", 10);
  timer_ = this->create_wall_timer(10s, std::bind(&PathRecorder::timer_callback, this));

  tf_to_wgs84_ = std::make_unique<nav_util::TfToWgs84>(shared_from_this());

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PathRecorder::on_activate(const rclcpp_lifecycle::State& state)
{
  if (tf_to_wgs84_->lookup_tf_to_wgs84(path_.wgs84_anchor_lat, path_.wgs84_anchor_lon, path_.wgs84_anchor_alt) == false)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "lookup_tf_to_wgs84 failed");
    return LNI::CallbackReturn::FAILURE;
  }

  LifecycleNode::on_activate(state);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/loc/odom", 10,
                                                                 std::bind(&PathRecorder::odom_callback, this, _1));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PathRecorder::on_deactivate(const rclcpp_lifecycle::State& state)
{
  odom_sub_.reset();

  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO_STREAM(this->get_logger(), "save path in ./path_performed.json");
  path_.filtering(1);
  path_.save("path_performed.json", "work_performed", path_vehicle_name_);
  path_.clear();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PathRecorder::on_cleanup(const rclcpp_lifecycle::State&)
{
  timer_.reset();
  path_pub_.reset();
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PathRecorder::on_shutdown(const rclcpp_lifecycle::State&)
{
  return LNI::CallbackReturn::SUCCESS;
}

void PathRecorder::timer_callback()
{
  path_.save("path_performed.json", "work_performed", path_vehicle_name_);
}

void PathRecorder::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double yaw;

  nav_util::quaternion_to_yaw(msg->pose.pose.orientation, yaw);

  path_.add_point(msg->pose.pose.position.x, msg->pose.pose.position.y);

  double s = std::round(msg->twist.twist.linear.x * 10) / 10.0;

  if (fabs(s) < 0.1)
  {
    s = copysign(0.1, msg->twist.twist.linear.x);
  }

  path_.set_speed_for_last_point(s);
  path_.set_tools_for_last_point(tools_state_);
}
}  // namespace nav_path_recorder

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav_path_recorder::PathRecorder)
