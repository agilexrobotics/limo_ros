#ifndef LIMON_BASE_NODE_H
#define LIMON_BASE_NODE_H
namespace agx {
struct LimonParams {
  static constexpr double max_steer_angle = 30.0;  // degree
  static constexpr double track = 0.58306;    // m (left right wheel distance)
  static constexpr double wheelbase = 0.498;  // m (front rear wheel distance)
  static constexpr double wheel_radius = 0.165;  // m

  static constexpr double max_linear_speed = 1.5;    // m/s
  static constexpr double max_angular_speed = 0.78;  // rad/s
  static constexpr double max_speed_cmd = 10.0;      // rad/s
};
}  // namespace agx
#endif