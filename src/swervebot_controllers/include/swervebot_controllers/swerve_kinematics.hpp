#ifndef SWERVEBOT_CONTROLLERS__SWERVE_KINEMATICS_HPP_
#define SWERVEBOT_CONTROLLERS__SWERVE_KINEMATICS_HPP_

#include <algorithm>
#include <array>
#include <cmath>

namespace swervebot_controllers
{
// Estimate [vx, vy, yaw_rate] from four rolling constraints and four no-side-slip constraints.
// All matrices have fixed stack storage: this routine is suitable for the controller's RT loop.
inline bool estimate_chassis_velocity(
  const double (& x)[4], const double (& y)[4], double radius, double offset_magnitude,
  const std::array<double, 4> & angles, const std::array<double, 4> & wheel_rates,
  const std::array<double, 4> & steering_rates, std::array<double, 3> & velocity)
{
  double normal[3][4]{};  // Augmented normal equations [A^T A | A^T b].
  const auto add_constraint = [&](const std::array<double, 3> & row, double value) {
      for (size_t j = 0; j < 3; ++j) {
        for (size_t k = 0; k < 3; ++k) {
          normal[j][k] += row[j] * row[k];
        }
        normal[j][3] += row[j] * value;
      }
    };
  for (size_t i = 0; i < 4; ++i) {
    const double c = std::cos(angles[i]), s = std::sin(angles[i]);
    const double offset = (i % 2 == 0 ? 1.0 : -1.0) * offset_magnitude;
    // t=(cos a,sin a), n=(-sin a,cos a); contact=pivot+offset*n.
    // Rolling: t.v_body + (x*sin a-y*cos a-offset)*yaw = r*wheel_rate+offset*a_dot.
    add_constraint({c, s, x[i] * s - y[i] * c - offset},
      radius * wheel_rates[i] + offset * steering_rates[i]);
    // The lateral offset cancels from the no-side-slip equation n.v_contact=0.
    add_constraint({-s, c, x[i] * c + y[i] * s}, 0.0);
  }
  // Partial-pivot Gaussian elimination on a 3x3 system avoids dynamic matrix workspaces.
  double scale = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    scale = std::max(scale, std::abs(normal[i][i]));
  }
  for (size_t column = 0; column < 3; ++column) {
    size_t pivot = column;
    for (size_t row = column + 1; row < 3; ++row) {
      if (std::abs(normal[row][column]) > std::abs(normal[pivot][column])) {pivot = row;}
    }
    if (!std::isfinite(normal[pivot][column]) ||
      std::abs(normal[pivot][column]) <= 1e-12 * std::max(scale, 1.0)) {return false;}
    for (size_t j = column; j < 4; ++j) {
      std::swap(normal[pivot][j], normal[column][j]);
    }
    for (size_t row = column + 1; row < 3; ++row) {
      const double factor = normal[row][column] / normal[column][column];
      for (size_t j = column; j < 4; ++j) {
        normal[row][j] -= factor * normal[column][j];
      }
    }
  }
  for (int row = 2; row >= 0; --row) {
    double value = normal[row][3];
    for (int j = row + 1; j < 3; ++j) {
      value -= normal[row][j] * velocity[j];
    }
    velocity[row] = value / normal[row][row];
    if (!std::isfinite(velocity[row])) {return false;}
  }
  return true;
}
}  // namespace swervebot_controllers
#endif  // SWERVEBOT_CONTROLLERS__SWERVE_KINEMATICS_HPP_
