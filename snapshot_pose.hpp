#pragma once
#include <array>
#include <cstdint>
#include <vector>
#include <pros/distance.hpp>
#include "snapshot_pose/collision_map.hpp"


// Put this in the include/snapshot_pose folder

namespace snapshot_pose {

// One physical distance sensor on the robot.
struct DistanceSensorConfig {
  pros::Distance* dev = nullptr;

  // Mount position relative to ROBOT CENTER in inches:
  // +x_right is robot-right, +y_fwd is robot-forward.
  float x_right_in = 0.0f;
  float y_fwd_in   = 0.0f;

  // Sensor facing direction relative to ROBOT FORWARD, in degrees:
  // 0 = forward, +90 = right, 180 = back, -90 = left.
  float rel_deg = 0.0f;

  // If nonzero, overrides SnapshotConfig::field_mask for THIS sensor.
  // Example: MAP_PERIMETER only for a low-mounted sensor.
  std::uint32_t field_mask_override = 0u;


  // Optional gating (leave defaults unless you need to reject game pieces).
  bool use_confidence_gate = true;
  int  min_confidence      = 35;  // (0..63); only applied when distance > 200mm
  
  bool use_velocity_gate   = false;
  double max_abs_vel_mps   = 0.25;

  bool use_object_size_gate = false;
  int  min_object_size      = 60;
};

struct SnapshotConfig {
  std::uint32_t field_mask = DEFAULT_FIELD_MASK;

  // How many candidate segment-hits to consider per sensor (K).
  // K=1 = fastest; K=2 or 3 = far more reliable if you enable interior objects.
  int candidates_per_sensor = 2;

  // Sampling the distance sensor and using the median reduces outliers.
  int samples = 7;
  int sample_delay_ms = 10;

  // VEX spec: range 20..2000mm. Below 200mm, ~±15mm; above 200mm, ~5%. :contentReference[oaicite:1]{index=1}
  float min_range_in = 0.79f;    // 20mm
  float max_range_in = 78.74f;   // 2000mm

  // Reject solutions if residuals are bad (3-sigma-ish per sensor).
  float max_chi2_per_sensor = 9.0f;
};

struct SnapshotResult {
  bool ok = false;

  float x_in = 0.0f;
  float y_in = 0.0f;
  float heading_deg = 0.0f; // JAR convention: 0 = +Y, clockwise-positive :contentReference[oaicite:2]{index=2}

  int used_sensors = 0;
  float chi2 = 0.0f;
};

// You provide your Odom type; we only need set_position().
template <typename OdomT>
SnapshotResult snapshot_setpose(
  OdomT& odom,
  const std::vector<DistanceSensorConfig>& sensors,
  const SnapshotConfig& cfg,
  float heading_deg_jar,
  float forward_tracker_in,
  float sideways_tracker_in,
  float odom_guess_x_in,
  float odom_guess_y_in
);

} // namespace snapshot_pose

#include "snapshot_pose/snapshot_pose_impl.hpp"
