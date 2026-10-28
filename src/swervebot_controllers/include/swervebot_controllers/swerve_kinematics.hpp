#ifndef SWERVEBOT_CONTROLLERS__SWERVE_KINEMATICS_HPP_
#define SWERVEBOT_CONTROLLERS__SWERVE_KINEMATICS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace swervebot_controllers
{
// Geometry uses body x forward, y left; offset is along the wheel's local left normal.
// Caller owns all storage; these value types and functions have no ROS dependencies.
struct ModuleGeometry
{
  double x, y, radius, offset;
  bool continuous;
  double minimum, maximum, velocity_limit;
};
struct ModuleState
{
  double angle = 0.0, wheel_rate = 0.0, steering_rate = 0.0;
};
struct ModuleTarget
{
  double angle = 0.0, wheel_rate = 0.0;
};

inline bool steering_in_range(const ModuleGeometry & module, double angle, double tolerance)
{
  return module.continuous ||
         (angle >= module.minimum - tolerance && angle <= module.maximum + tolerance);
}

inline double bounded_angle(const ModuleGeometry & module, double angle)
{
  return module.continuous ? angle : std::clamp(angle, module.minimum, module.maximum);
}

// All headings angle+k*pi describe the same rolling line. Select the closest feasible one,
// allowing signed drive speed to reverse rolling direction rather than rotating unnecessarily.
inline bool select_heading(
  const ModuleGeometry & module, double angle, double measured, double & selected)
{
  double turns = std::floor((measured - angle) / M_PI + 0.5);
  if (!module.continuous) {
    const double first = std::ceil((module.minimum - angle) / M_PI);
    const double last = std::floor((module.maximum - angle) / M_PI);
    if (first > last) {return false;}
    turns = std::clamp(turns, first, last);
  }
  selected = bounded_angle(module, angle + turns * M_PI);
  return std::isfinite(selected);
}

inline bool inverse_kinematics(
  const ModuleGeometry & module, const ModuleState & state,
  double vx, double vy, double omega, double dt, ModuleTarget & target)
{
  target = {bounded_angle(module, state.angle), 0.0};
  if (vx == 0.0 && vy == 0.0 && omega == 0.0) {return true;}
  // Velocity at the steering pivot: v_pivot = v_body + omega cross r_pivot.
  const double px = vx - omega * module.y, py = vy + omega * module.x;
  if (px != 0.0 || py != 0.0) {
    if (!select_heading(module, std::atan2(py, px), state.angle, target.angle)) {return false;}
  }
  // Slew positions before hardware writes, with numerical headroom for downstream limiters.
  const double step = std::max(module.velocity_limit * dt - 1e-9, 0.0);
  target.angle = bounded_angle(module,
    std::clamp(target.angle, state.angle - step, state.angle + step));
  // Project onto measured rolling direction while steering catches up. Lateral offset adds
  // -offset*(body yaw rate + steering rate) to wheel-center rolling velocity.
  target.wheel_rate = (px * std::cos(state.angle) + py * std::sin(state.angle) -
    module.offset * (omega + state.steering_rate)) / module.radius;
  return std::isfinite(target.angle) && std::isfinite(target.wheel_rate);
}

// Accumulate rolling and no-side-slip constraints into a fixed 3x3 least-squares system.
// Module buffers are allocated by the caller during configuration, never by this solver.
inline bool estimate_chassis_velocity(
  const ModuleGeometry * modules, const ModuleState * states, std::size_t count,
  std::array<double, 3> & velocity)
{
  double normal[3][4]{};  // Augmented normal equations [A^T A | A^T b].
  const auto add_constraint = [&](const std::array<double, 3> & row, double value) {
      for (std::size_t j = 0; j < 3; ++j) {
        for (std::size_t k = 0; k < 3; ++k) {
          normal[j][k] += row[j] * row[k];
        }
        normal[j][3] += row[j] * value;
      }
    };
  for (std::size_t i = 0; i < count; ++i) {
    const auto & module = modules[i];
    const auto & state = states[i];
    const double c = std::cos(state.angle), s = std::sin(state.angle);
    // t=(cos a,sin a), n=(-sin a,cos a); contact=pivot+offset*n.
    // Rolling: t.v_body + (x*sin a-y*cos a-offset)*yaw = r*wheel_rate+offset*a_dot.
    add_constraint({c, s, module.x * s - module.y * c - module.offset},
      module.radius * state.wheel_rate + module.offset * state.steering_rate);
    // The lateral offset cancels from the no-side-slip equation n.v_contact=0.
    add_constraint({-s, c, module.x * c + module.y * s}, 0.0);
  }
  // Partial-pivot Gaussian elimination on a 3x3 system avoids dynamic matrix workspaces.
  double scale = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    scale = std::max(scale, std::abs(normal[i][i]));
  }
  for (std::size_t column = 0; column < 3; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1; row < 3; ++row) {
      if (std::abs(normal[row][column]) > std::abs(normal[pivot][column])) {pivot = row;}
    }
    if (!std::isfinite(normal[pivot][column]) ||
      std::abs(normal[pivot][column]) <= 1e-12 * std::max(scale, 1.0)) {return false;}
    for (std::size_t j = column; j < 4; ++j) {
      std::swap(normal[pivot][j], normal[column][j]);
    }
    for (std::size_t row = column + 1; row < 3; ++row) {
      const double factor = normal[row][column] / normal[column][column];
      for (std::size_t j = column; j < 4; ++j) {
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
