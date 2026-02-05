#pragma once
#include "snapshot_pose/snapshot_pose.hpp"
#include <vector>

namespace snapshot_pose {

// Library-agnostic callbacks for reading odom state and applying pose.
// Set once from your project (LemLib, EZ-Template, custom, etc.).
struct SnapshotPoseRuntime {
  void* user_data = nullptr;

  // Required callbacks:
  float (*get_heading_deg)(void* user_data) = nullptr;
  float (*get_guess_x_in)(void* user_data) = nullptr;
  float (*get_guess_y_in)(void* user_data) = nullptr;
  void (*apply_pose)(void* user_data,
                     float x_in,
                     float y_in,
                     float heading_deg,
                     float forward_tracker_in,
                     float sideways_tracker_in) = nullptr;

  // Optional callbacks (defaults to 0.0 when null).
  float (*get_forward_tracker_in)(void* user_data) = nullptr;
  float (*get_sideways_tracker_in)(void* user_data) = nullptr;
};

// Example callback mappings:
//
// LemLib:
//   get_heading_deg:    const auto p = chassis.getPose(false, false); return p.theta;
//   get_guess_x_in/y_in const auto p = chassis.getPose(false, false); return p.x / p.y;
//   apply_pose:         chassis.setPose(x_in, y_in, heading_deg);
//
// JAR:
//   get_heading_deg:    return odom.orientation_deg;
//   get_guess_x_in/y_in return odom.X_position / odom.Y_position;
//   apply_pose:         odom.set_position(x_in, y_in, heading_deg, fwd, side);

// Runtime wiring + optional config overrides.
// snapshot_bindings_set_runtime(...) must be called before snapshot_setpose_quadrant().
bool snapshot_bindings_set_runtime(const SnapshotPoseRuntime& runtime);

// Replace the full sensor list. Each sensor can use its own field mask via
// DistanceSensorConfig::field_mask_override.
//
// Sensor geometry fields:
//   x_right_in, y_fwd_in, rel_deg
//
// Per-sensor collision filtering:
//   field_mask_override = MAP_PERIMETER | MAP_LONG_GOALS_ALL; // example
void snapshot_bindings_set_sensors(const std::vector<DistanceSensorConfig>& sensors);

// Update a single sensor entry in-place. Returns false if index is out of range.
bool snapshot_bindings_update_sensor(std::size_t index, const DistanceSensorConfig& sensor);

// Convenience helper: set only one sensor's raycast mask. Returns false if index is out of range.
// Use 0 to fall back to SnapshotConfig::field_mask.
bool snapshot_bindings_set_sensor_mask(std::size_t index, std::uint32_t mask_override);

void snapshot_bindings_set_config(const SnapshotConfig& cfg);
void snapshot_bindings_reset_defaults();

// One-call API for auton.
// If ok==false, pose is not modified.
// If ok==true, pose is applied through SnapshotPoseRuntime::apply_pose.
SnapshotResult snapshot_setpose_quadrant(Quadrant q = Quadrant::ANY);

} // namespace snapshot_pose
