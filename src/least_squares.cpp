#include "nav_path_recorder/least_squares.hpp"

namespace nav_path_recorder
{
/**
 * Search the closest curve for the X and Y vector (faster than the other)
 */
bool LeastSquares::compute_2nd_degree_polynomial_regression(const Eigen::VectorXd& X, const Eigen::VectorXd& Y,
                                                            Eigen::Vector3d& result_coef)
{
  ide = 0;
  MCx1 = 0, MCx2 = 0, MCx3 = 0, MCx4 = 0;
  MCy1 = 0, MCxy = 0, MCx2y = 0;

  Xoff = X[0];

  for (int i = 0; i < X.size(); i++)
  {
    Xtemp = X[i] - Xoff;
    Ytemp = Y[i];
    ide = ide + 1;
    MCx1 = MCx1 + Xtemp;
    MCx2 = MCx2 + Xtemp * Xtemp;
    MCx3 = MCx3 + Xtemp * Xtemp * Xtemp;
    MCx4 = MCx4 + Xtemp * Xtemp * Xtemp * Xtemp;
    MCy1 = MCy1 + Ytemp;
    MCxy = MCxy + Xtemp * Ytemp;
    MCx2y = MCx2y + Xtemp * Xtemp * Ytemp;
  }

  passage(0, 0) = ide;
  passage(0, 1) = MCx1;
  passage(0, 2) = MCx2;
  passage(1, 0) = MCx1;
  passage(1, 1) = MCx2;
  passage(1, 2) = MCx3;
  passage(2, 0) = MCx2;
  passage(2, 1) = MCx3;
  passage(2, 2) = MCx4;

  F(0) = MCy1;
  F(1) = MCxy;
  F(2) = MCx2y;

  bool success{ true };
  passage.computeInverseWithCheck(passage_inv, success);

  result_coef = passage_inv * F;
  result_coef(1) = result_coef(1) - 2 * result_coef(2) * Xoff;
  result_coef(0) = result_coef(0) - result_coef(1) * Xoff - result_coef(2) * Xoff * Xoff;

  return success;
}
}  // namespace nav_path_recorder
