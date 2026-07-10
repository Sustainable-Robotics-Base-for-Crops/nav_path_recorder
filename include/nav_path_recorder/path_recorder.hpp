// Copyright 2026 SABI AGRI

#ifndef NAV_PATH_RECORDER__PATH_RECORDER_HPP_
#define NAV_PATH_RECORDER__PATH_RECORDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_util/path2d.hpp"
#include "nav_util/tf_to_wgs84.hpp"
#include "nav_msgs/msg/odometry.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace nav_path_recorder
{
class PathRecorder : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PathRecorder(const rclcpp::NodeOptions& options);

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

protected:
  void timer_callback();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  std::string path_vehicle_name_;
  bool tools_state_{ false };

  nav_util::Path2D path_;
  std::unique_ptr<nav_util::TfToWgs84> tf_to_wgs84_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
}  // namespace nav_path_recorder

#endif  // NAV_PATH_RECORDER__PATH_RECORDER_HPP_
