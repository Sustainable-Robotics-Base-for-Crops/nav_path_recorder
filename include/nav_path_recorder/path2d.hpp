// Copyright 2026 SABI AGRI

#ifndef NAV_PATH_RECORDER__PATH2D_HPP_
#define NAV_PATH_RECORDER__PATH2D_HPP_

#include "rclcpp/clock.hpp"
#include "nav_path_recorder/least_squares.hpp"
#include "nav_msgs/msg/path.hpp"
#include <Eigen/Dense>
#include <map>

typedef Eigen::Vector2d Point2D;

namespace nav_path_recorder
{
class Path2D
{
public:
  /// Constructor
  Path2D();
  ~Path2D();

  /**
   * load a path
   * @param[in] file_path : yaml file with X,Y position to load
   * @param[in] allow_replay_negative_speed : true if negative speed allowed in path
   * @result return true if success
   */
  bool load(const std::string& file_path, bool allow_replay_negative_speed = false);

  // clear path
  void clear();

  /**
   * Add a point to the path
   * @param[in] X : X coordinate (EAST) of point to add
   * @param[in] Y : Y coordinate (NORTH) of point to add
   * @result return true if point added
   */
  bool add_point(const double X, const double Y);

  /**
   * Add a point to the path
   * @param[in] X : X coordinate (EAST) of point to add
   * @param[in] Y : Y coordinate (NORTH) of point to add
   * @param[in] heading : heading at point to add (zero pointing EAST)
   * @result return true if point added
   */
  bool add_point(const double X, const double Y, const double heading);

  /**
   * set_active_window set the window length which is used to compute
   * phi_ref and curvature on traj arround a given point
   * @param[in] active_window length in meters
   */
  bool set_active_window(const double& active_window)
  {
    if (active_window > 0. && active_window < 100.)
    {
      active_window_ = active_window / 2;
      return true;
    }
    return false;
  }

  bool set_distance_min_between_points(const double dist_min)
  {
    if (dist_min >= 0.0 && dist_min < 1.0)
    {
      dist_min_ = dist_min;
      return true;
    }
    return false;
  }

  /**
   * Set desired tools state for last point on path
   * @param[in] tools : tools state on point to add (true: tools ON ; false: tools OFF)
   */
  void set_tools_for_last_point(const bool tools);

  /**
   * Set desired control speed for last point on path
   * * @param[in] speed
   */
  void set_speed_for_last_point(const double speed);

  /**
   * Set desired course on last point on path
   * * @param[in] course
   */
  void set_course_for_last_point(const double course);

  /**
   * Smooth the recorded path
   * @param[in] degree : filtering degree
   */
  void filtering(int degree);

  /**
   * Save a path
   * @param[in] file_path : file_path where to save the path
   * @param[in] file_type : file_type
   * @param[in] vehicle_id
   */
  void save(const std::string& file_path, const std::string& file_type, const std::string& vehicle_id);

  /**
   * Is path has enough point to follow it ?
   * @return true if enough point in path
   */
  bool has_point()
  {
    return nb_point_ > 10;
  }

  void get_path(nav_msgs::msg::Path& path);

  void translate_path(double offset_lateral, double direction);
  void set_offsets(double offset, double orientation);

  double wgs84_anchor_lat{ std::numeric_limits<double>::quiet_NaN() };
  double wgs84_anchor_lon{ std::numeric_limits<double>::quiet_NaN() };

protected:
  /**
   * Get desired control speed at path index
   * @param[in] index : path index of point
   * @param[out] speed : desired control speed
   * @result return true if success
   */
  bool get_speed_at_index(const int index, double& speed);
  /**
   * Get desired tools state at path index
   * @param[in] index : path index of point
   * @param[out] tools : desired tools state
   * @result return true if success
   */
  bool get_tools_at_index(const int index, bool& tools);
  /**
   * Get desired course at path index
   * @param[in] index : path index of point
   * @param[out] course : desired course
   * @result return true if success
   */
  bool get_course_at_index(const int index, double& course);

  bool compute_curvature_on_path_with_index(const int current_index, double& curvature);
  bool select_near_points(const int index, int& i_begin, int& i_end);
  double compute_curvature(const double& current_abs_curv, const Eigen::Vector3d& curve_coef_x,
                           const Eigen::Vector3d& curve_coef_y);

  // Size of window for curvature computation
  double active_window_{ 1.5 };
  /// Minimal distance in meters between points, to add a new point in the traj
  double dist_min_{ 0.09 };
  // X_ : x positions pointing east
  // Y_ : y position pointing north
  // S_ : curvilinear abscissia
  Eigen::VectorXd X_, Y_, S_;
  int nb_point_{ 0 };
  Eigen::VectorXd speed_, tools_;
  // map(index, course)
  std::map<int, double> course_map_;

  double translate_offset_{ 0. }, translate_orientation_{ 0. };
  double heading_prev_{ std::numeric_limits<double>::quiet_NaN() };
  LeastSquares ls_;

  // Logging
  rclcpp::Clock clock_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Because of Point2D -> Vector2d
};
}  // namespace nav_path_recorder

#endif  // NAV_PATH_RECORDER__PATH2D_HPP_
