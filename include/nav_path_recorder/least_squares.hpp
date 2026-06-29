// Copyright 2026 SABI AGRI

#ifndef NAV_PATH_RECORDER__LEAST_SQUARES_HPP_
#define NAV_PATH_RECORDER__LEAST_SQUARES_HPP_

#include <Eigen/Dense>

namespace nav_path_recorder
{
class LeastSquares
{
public:
  LeastSquares()
  {
  }
  ~LeastSquares()
  {
  }

  /**
   * Search the closest curve for the X and Y vector (faster than the other)
   * @param[in] X vector of measured data; X and Y must be of the same size
   * @param[in] Y vector of measured data; X and Y must be of the same size
   * @param[out] result_coef = {c, b, a}, three parameter of the plynomial curve : Y = aX² + bX + c
   * @return true when sucess
   */
  bool compute_2nd_degree_polynomial_regression(const Eigen::VectorXd& X, const Eigen::VectorXd& Y,
                                                Eigen::Vector3d& result_coef);

private:
  double ide;
  double MCx1, MCx2, MCx3, MCx4;
  double MCy1, MCxy, MCx2y;

  double Xoff;
  double Xtemp, Ytemp;

  Eigen::Vector3d F;
  Eigen::Matrix3d passage;
  Eigen::Matrix3d passage_inv;
};
}  // namespace nav_path_recorder

#endif  // NAV_PATH_RECORDER__LEAST_SQUARES_HPP_
