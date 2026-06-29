#include "nav_path_recorder/path2d.hpp"
#include "rclcpp/logging.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "nlohmann/json.hpp"
#include "angles/angles.h"
#include <iostream>
#include <fstream>

using json = nlohmann::json;

namespace nav_path_recorder
{
Path2D::Path2D()
{
  clear();
}

Path2D::~Path2D()
{
}

bool Path2D::load(const std::string& file_path, bool allow_replay_negative_speed)
{
  GeographicLib::LocalCartesian enu_loc(wgs84_anchor_lat, wgs84_anchor_lon);
  std::ifstream f(file_path);
  json data = json::parse(f);
  if (data["version"] != "1" && data["version"] != "3")
  {
    std::cout << "Only accept path format version 1 or 3 ; actual version " << data["version"] << std::endl;
    return false;
  }

  if (data["file_type"] != "mission_order")
  {
    std::cout << "Only accept path format file_type mission_order ; actual file_type " << data["file_type"]
              << std::endl;
    return false;
  }

  auto origin = data["origin"];
  if (origin["type"] != "WGS84")
  {
    std::cout << "Only accept path format with origin type WGS84 ; actual origin type " << origin["type"] << std::endl;
    return false;
  }
  auto wgs84_anchor = origin["coordinates"];
  double path_wgs84_anchor_lat;
  double path_wgs84_anchor_lon;
  if (data["version"] == "1")
  {
    path_wgs84_anchor_lat = wgs84_anchor[0].get<double>();
    path_wgs84_anchor_lon = wgs84_anchor[1].get<double>();
  }
  else
  {
    path_wgs84_anchor_lat = wgs84_anchor["lat"].get<double>();
    path_wgs84_anchor_lon = wgs84_anchor["lon"].get<double>();
  }
  GeographicLib::LocalCartesian enu_path(path_wgs84_anchor_lat, path_wgs84_anchor_lon);

  double lat_rev, lon_rev, h_rev;
  double x_loc, y_loc, z_loc;

  auto points = data["points"];
  clear();
  for (const auto& path : points)
  {
    std::map<std::string, std::size_t> col_indexes;
    std::size_t i = 0;
    for (auto const& col : path["columns"])
    {
      col_indexes[col] = i;
      ++i;
    }
    bool speed_present = col_indexes.count(std::string("speed")) > 0;
    if (speed_present)
    {
      speed_.resize(path["values"].size());
    }
    bool tools_present = col_indexes.count("working_zone") > 0;
    if (tools_present)
    {
      tools_.resize(path["values"].size());
    }
    bool punctual_present = col_indexes.count(std::string("punctual")) > 0;
    for (const auto& point : path["values"])
    {
      enu_path.Reverse(point[col_indexes["x"]].get<double>(), point[col_indexes["y"]].get<double>(), 0.0, lat_rev,
                       lon_rev, h_rev);
      enu_loc.Forward(lat_rev, lon_rev, h_rev, x_loc, y_loc, z_loc);
      add_point(x_loc, y_loc);
      if (speed_present)
      {
        double speed = point[col_indexes["speed"]].get<double>();
        if (allow_replay_negative_speed == false && speed < 0.0)
        {
          clear();
          RCLCPP_ERROR_STREAM_THROTTLE(rclcpp::get_logger("path2D"), clock_, 1000,
                                       "ERROR load path ; path with neagtive speed consign not allowed");
          return false;
        }
        set_speed_for_last_point(speed);
      }
      if (tools_present)
      {
        bool tools = point[col_indexes["working_zone"]].get<int>();
        set_tools_for_last_point(tools);
      }
      if (punctual_present)
      {
        auto punctual = point[col_indexes["punctual"]];
        if (punctual.contains("course"))
        {
          double course = punctual["course"].get<double>();
          std::cout << "course " << course << std::endl;
          set_course_for_last_point(course);
        }
      }
    }
    // Only load the first path
    break;
  }

  return has_point();
}

void Path2D::clear()
{
  X_.resize(0);
  Y_.resize(0);
  nb_point_ = 0;

  S_.resize(1);
  S_[0] = 0;

  speed_.resize(0);
  tools_.resize(0);
  translate_offset_ = 0.0;
  translate_orientation_ = 0.0;
  course_map_.clear();
}

bool Path2D::add_point(const double X, const double Y)
{
  double ds = 0;
  Point2D new_point;
  new_point.x() = X - translate_offset_ * sin(translate_orientation_);
  new_point.y() = Y + translate_offset_ * cos(translate_orientation_);

  if (nb_point_ > 0)
  {
    Point2D prev_point;
    prev_point << X_[nb_point_ - 1], Y_[nb_point_ - 1];
    ds = (new_point - prev_point).norm();
  }
  else
  {
    ds = dist_min_;
  }

  if (ds >= dist_min_)
  {
    ++nb_point_;
    if (nb_point_ > X_.size())
    {
      X_.conservativeResize(nb_point_);
      Y_.conservativeResize(nb_point_);
      S_.conservativeResize(nb_point_);
    }
    X_[nb_point_ - 1] = new_point.x();
    Y_[nb_point_ - 1] = new_point.y();
    if (nb_point_ > 1)
    {
      // Abscisse curviligne du nouveau point : ajout de la distance parcourues entre les deux derniers points
      S_[nb_point_ - 1] = S_[nb_point_ - 2] + ds;
    }
    return true;
  }
  return false;
}

bool Path2D::add_point(const double X, const double Y, const double heading)
{
  bool success = add_point(X, Y);
  if (success)
  {
    if (std::isnan(heading_prev_))
    {
      heading_prev_ = heading;
    }
    double angle_diff = angles::shortest_angular_distance(heading, heading_prev_);
    if (fabs(angle_diff) > M_PI / 6.)
    {
      course_map_.insert_or_assign(nb_point_ - 1, heading);
    }
    heading_prev_ = heading;
  }
  return success;
}

void Path2D::set_tools_for_last_point(const bool tools)
{
  if (nb_point_ > tools_.size())
  {
    tools_.conservativeResize(nb_point_);
  }
  tools_[nb_point_ - 1] = tools;
}

void Path2D::set_speed_for_last_point(const double speed)
{
  if (nb_point_ > speed_.size())
  {
    speed_.conservativeResize(nb_point_);
  }
  if (fabs(speed) > 0.01)
  {
    speed_[nb_point_ - 1] = speed;
  }
  else
  {
    speed_[nb_point_ - 1] = 0.0;
  }
}

void Path2D::set_course_for_last_point(const double course)
{
  if (nb_point_ > 0)
  {
    course_map_.insert_or_assign(nb_point_ - 1, course);
  }
}

bool Path2D::get_speed_at_index(const int index, double& speed)
{
  if (speed_.size() > index)
  {
    speed = speed_[index];
    return true;
  }
  else
  {
    return false;
  }
}

bool Path2D::get_tools_at_index(const int index, bool& tools)
{
  if (tools_.size() > index)
  {
    tools = tools_[index];
    return true;
  }
  else
  {
    return false;
  }
}

bool Path2D::get_course_at_index(const int index, double& course)
{
  auto search = course_map_.find(index);
  if (search != course_map_.end())
  {
    course = search->second;
    return true;
  }
  else
  {
    return false;
  }
}

void Path2D::filtering(int degree)
{
  if (has_point())
  {
    std::vector<double> X_tmp;
    X_tmp.resize(nb_point_);
    std::vector<double> Y_tmp;
    Y_tmp.resize(nb_point_);
    for (int j = 0; j < degree; ++j)
    {
      X_tmp[0] = X_[0];
      Y_tmp[0] = Y_[0];
      X_tmp[nb_point_ - 1] = X_[nb_point_ - 1];
      Y_tmp[nb_point_ - 1] = Y_[nb_point_ - 1];

      for (int i = 1; i <= nb_point_ - 2; ++i)
      {
        X_tmp[i] = (X_[i - 1] + X_[i] + X_[i + 1]) / 3;
        Y_tmp[i] = (Y_[i - 1] + Y_[i] + Y_[i + 1]) / 3;
      }
      for (int i = 1; i <= nb_point_ - 2; ++i)
      {
        X_[i] = (X_tmp[i - 1] + X_tmp[i] + X_tmp[i + 1]) / 3;
        Y_[i] = (Y_tmp[i - 1] + Y_tmp[i] + Y_tmp[i + 1]) / 3;
      }
    }
  }
}

void Path2D::save(const std::string& file_path, const std::string& file_type, const std::string& vehicle_id)
{
  json j;
  j["version"] = "3";
  if (file_type == "mission_order" || file_type == "work_performed")
  {
    j["file_type"] = file_type;
  }
  else
  {
    j["file_type"] = "mission_order";
  }
  j["vehicle_id"] = vehicle_id;
  j["origin"]["type"] = "WGS84";
  if (j["version"] == "1")
  {
    j["origin"]["coordinates"][0] = wgs84_anchor_lat;
    j["origin"]["coordinates"][1] = wgs84_anchor_lon;
  }
  else
  {
    j["origin"]["coordinates"]["lat"] = wgs84_anchor_lat;
    j["origin"]["coordinates"]["lon"] = wgs84_anchor_lon;
  }
  j["points"] = json::array();
  j["points"][0]["columns"] = { "x", "y" };
  if (speed_.size() > 0)
  {
    j["points"][0]["columns"].push_back("speed");
  }
  if (tools_.size() > 0)
  {
    j["points"][0]["columns"].push_back("working_zone");
  }
  j["points"][0]["columns"].push_back("punctual");
  double speed{ 0.0 };
  bool tools{ false };
  for (int i = 0; i < nb_point_; ++i)
  {
    std::vector<double> point;
    // Millimeter precision
    point.push_back(round(X_[i] * 1000) / 1000);
    point.push_back(round(Y_[i] * 1000) / 1000);
    if (get_speed_at_index(i, speed))
    {
      point.push_back(speed);
    }
    if (get_tools_at_index(i, tools))
    {
      point.push_back(tools);
    }
    j["points"][0]["values"].push_back(point);
    double course;
    if (get_course_at_index(i, course))
    {
      j["points"][0]["values"].back().push_back({ { "course", course } });
    }
    else
    {
      j["points"][0]["values"].back().push_back({});
    }
  }

  std::ofstream file(file_path.c_str());
  file << j.dump() << std::endl;
  file.close();
}

void Path2D::get_path(nav_msgs::msg::Path& path)
{
  path.poses.reserve(nb_point_);
  for (int i = 0; i < nb_point_; ++i)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = X_[i];
    pose.pose.position.y = Y_[i];
    path.poses.push_back(pose);
  }
}

void Path2D::translate_path(double offset, double orientation)
{
  for (int i = 0; i < nb_point_; ++i)
  {
    X_[i] = X_[i] + translate_offset_ * sin(translate_orientation_);
    Y_[i] = Y_[i] - translate_offset_ * cos(translate_orientation_);
  }

  translate_offset_ = offset;
  translate_orientation_ = orientation;

  for (int i = 0; i < nb_point_; ++i)
  {
    X_[i] = X_[i] - translate_offset_ * sin(translate_orientation_);
    Y_[i] = Y_[i] + translate_offset_ * cos(translate_orientation_);
  }
}

void Path2D::set_offsets(double offset, double orientation)
{
  translate_offset_ = offset;
  translate_orientation_ = orientation;
}

bool Path2D::compute_curvature_on_path_with_index(const int current_index, double& curvature)
{
  int i_d = 0, i_f = 0;
  if (select_near_points(current_index, i_d, i_f) == false)
  {
    RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("path2D"), clock_, 1000,
                                "ERROR compute_curvature_on_path_with_index ; select_near_points");
    return false;
  }

  assert(i_d <= i_f);
  assert(i_f < X_.size());

  bool result_regression{ true };
  Eigen::Vector3d result_x, result_y;
  // Y=ay+by*T+cy*T^2
  result_regression = ls_.compute_2nd_degree_polynomial_regression(S_.segment(i_d, i_f - i_d + 1),
                                                                   Y_.segment(i_d, i_f - i_d + 1), result_y);
  //  X=ax+bx*T+cx*T^2
  result_regression = ls_.compute_2nd_degree_polynomial_regression(S_.segment(i_d, i_f - i_d + 1),
                                                                   X_.segment(i_d, i_f - i_d + 1), result_x);
  if (result_regression == false)
  {
    RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("path2D"), clock_, 1000,
                                "ERROR compute_curvature_on_path_with_index ; "
                                "compute_2nd_degree_polynomial_regression");
    return false;
  }

  curvature = compute_curvature(S_[current_index], result_x, result_y);

  if (current_index >= (nb_point_ - 5))
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool Path2D::select_near_points(const int index, int& i_begin, int& i_end)
{
  bool success = true;
  Point2D point_d, point_current;
  double length = 0.0;
  double speed_at_index, speed_at_i;
  bool speed_on_path = get_speed_at_index(index, speed_at_index);
  double course;

  point_d << X_[index], Y_[index];
  i_end = nb_point_ - 1;
  for (int i = index; i < nb_point_ - 1; i++)
  {
    point_current << X_[i], Y_[i];
    length += (point_d - point_current).norm();
    if (speed_on_path && get_speed_at_index(i, speed_at_i) &&
        (std::signbit(speed_at_i) != std::signbit(speed_at_index)))
    {
      i_end = i - 1;
      break;
    }
    if (length >= active_window_ || (get_course_at_index(i, course) && index != i))
    {
      i_end = i;
      break;
    }
    point_d = point_current;
  }

  point_d << X_[index], Y_[index];
  length = 0;
  i_begin = 0;
  for (int i = index; i > 0; i--)
  {
    point_current << X_[i], Y_[i];
    length += (point_d - point_current).norm();
    if (speed_on_path && get_speed_at_index(i, speed_at_i) &&
        (std::signbit(speed_at_i) != std::signbit(speed_at_index)))
    {
      i_begin = i + 1;
      break;
    }
    if (length >= active_window_ || get_course_at_index(i, course))
    {
      i_begin = i;
      break;
    }
    point_d = point_current;
  }
  // TODO rework i_begin i_end ; could lead to i_end == i_begin
  // how many point min we need to compute curvature ?
  // return false in case of failure
  if (i_begin == i_end)
  {
    if (i_end >= nb_point_ - 1)
    {
      i_begin--;
    }
    else
    {
      i_end++;
    }
  }
  if (i_begin < 0)
  {
    i_begin = 0;
  }
  if (i_end < 0)
  {
    i_end = 0;
  }
  return success;
}

double Path2D::compute_curvature(const double& current_abs_curv, const Eigen::Vector3d& curve_coef_x,
                                 const Eigen::Vector3d& curve_coef_y)
{
  double curvature{ 0 };
  if (current_abs_curv > 0)
  {
    double cy, by, cx, bx;
    double Xdot, Xdotdot, Ydot, Ydotdot, denominateur;
    cy = curve_coef_y[2];
    by = curve_coef_y[1];

    cx = curve_coef_x[2];
    bx = curve_coef_x[1];

    Xdot = bx + (2 * cx * current_abs_curv);
    Ydot = by + (2 * cy * current_abs_curv);
    Xdotdot = 2 * cx;
    Ydotdot = 2 * cy;
    denominateur = Xdot * Ydotdot - Ydot * Xdotdot;
    if (fabs(denominateur) > std::numeric_limits<double>::epsilon())
    {
      double tempo = sqrt(Xdot * Xdot + Ydot * Ydot);
      curvature = denominateur / (tempo * tempo * tempo);
    }
  }
  return curvature;
}
}  // namespace nav_path_recorder
