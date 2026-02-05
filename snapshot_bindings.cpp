#include "snapshot_pose/snapshot_bindings.hpp"

#include <algorithm>
#include <pros/lcd.hpp>

namespace snapshot_pose {
namespace {

// Default local devices. Replace with extern declarations if your project already
// owns these sensors elsewhere.
pros::Distance distFront(1);
pros::Distance distRight(2);

SnapshotConfig g_cfg{};
std::vector<DistanceSensorConfig> g_sensors;
SnapshotPoseRuntime g_runtime{};

bool g_init = false;
bool g_cfg_custom = false;
bool g_sensors_custom = false;

bool runtime_ready(const SnapshotPoseRuntime& runtime) {
  return runtime.get_heading_deg != nullptr &&
         runtime.get_guess_x_in != nullptr &&
         runtime.get_guess_y_in != nullptr &&
         runtime.apply_pose != nullptr;
}

void apply_quadrant_bias(float& guess_x, float& guess_y, Quadrant q, float margin_in) {
  if (q == Quadrant::ANY) return;

  const float mid = FIELD_SIZE_IN * 0.5f;
  const float margin = std::max(0.0f, margin_in);

  switch (q) {
    case Quadrant::BL:
      guess_x = std::min(guess_x, mid - margin);
      guess_y = std::min(guess_y, mid - margin);
      break;
    case Quadrant::BR:
      guess_x = std::max(guess_x, mid + margin);
      guess_y = std::min(guess_y, mid - margin);
      break;
    case Quadrant::TL:
      guess_x = std::min(guess_x, mid - margin);
      guess_y = std::max(guess_y, mid + margin);
      break;
    case Quadrant::TR:
      guess_x = std::max(guess_x, mid + margin);
      guess_y = std::max(guess_y, mid + margin);
      break;
    case Quadrant::ANY:
    default:
      break;
  }
}

void init_once() {
  if (g_init) return;
  g_init = true;

  if (!g_cfg_custom) {
    g_cfg = SnapshotConfig{};
    g_cfg.field_mask = MAP_PERIMETER;
    g_cfg.candidates_per_sensor = 1;
    g_cfg.samples = 5;
    g_cfg.sample_delay_ms = 35;
    g_cfg.max_chi2_per_sensor = 9.0f;
    g_cfg.quadrant_margin_in = 2.0f;
  }

  if (!g_sensors_custom) {
    g_sensors.clear();
    g_sensors.reserve(2);

    DistanceSensorConfig front{};
    front.dev = &distFront;
    front.x_right_in = 0.0f;
    front.y_fwd_in = 7.0f;
    front.rel_deg = 0.0f;
    front.field_mask_override = MAP_PERIMETER;
    front.use_confidence_gate = true;
    front.min_confidence = 35;
    g_sensors.push_back(front);

    DistanceSensorConfig right{};
    right.dev = &distRight;
    right.x_right_in = 7.0f;
    right.y_fwd_in = 0.0f;
    right.rel_deg = 90.0f;
    right.field_mask_override = MAP_PERIMETER | MAP_LONG_GOALS_ALL;
    right.use_confidence_gate = true;
    right.min_confidence = 35;
    g_sensors.push_back(right);
  }

  pros::lcd::print(6, "SnapshotPose init OK");
}

struct RuntimeOdomAdapter {
  SnapshotPoseRuntime runtime;

  void set_position(float x_in,
                    float y_in,
                    float heading_deg,
                    float forward_tracker_in,
                    float sideways_tracker_in) {
    runtime.apply_pose(runtime.user_data,
                       x_in,
                       y_in,
                       heading_deg,
                       forward_tracker_in,
                       sideways_tracker_in);
  }
};

} // namespace

bool snapshot_bindings_set_runtime(const SnapshotPoseRuntime& runtime) {
  if (!runtime_ready(runtime)) return false;
  g_runtime = runtime;
  return true;
}

void snapshot_bindings_set_sensors(const std::vector<DistanceSensorConfig>& sensors) {
  g_sensors = sensors;
  g_sensors_custom = true;
  g_init = false;
}

void snapshot_bindings_set_config(const SnapshotConfig& cfg) {
  g_cfg = cfg;
  g_cfg_custom = true;
  g_init = false;
}

void snapshot_bindings_reset_defaults() {
  g_cfg_custom = false;
  g_sensors_custom = false;
  g_cfg = SnapshotConfig{};
  g_sensors.clear();
  g_init = false;
}

SnapshotResult snapshot_setpose_quadrant(Quadrant q) {
  init_once();

  if (!runtime_ready(g_runtime)) {
    pros::lcd::print(6, "SnapshotPose runtime missing");
    return SnapshotResult{};
  }

  const float heading_deg = g_runtime.get_heading_deg(g_runtime.user_data);
  const float fwd_in = g_runtime.get_forward_tracker_in != nullptr
                           ? g_runtime.get_forward_tracker_in(g_runtime.user_data)
                           : 0.0f;
  const float side_in = g_runtime.get_sideways_tracker_in != nullptr
                            ? g_runtime.get_sideways_tracker_in(g_runtime.user_data)
                            : 0.0f;

  float guess_x = g_runtime.get_guess_x_in(g_runtime.user_data);
  float guess_y = g_runtime.get_guess_y_in(g_runtime.user_data);
  apply_quadrant_bias(guess_x, guess_y, q, g_cfg.quadrant_margin_in);

  RuntimeOdomAdapter odom{g_runtime};
  return snapshot_setpose(odom,
                          g_sensors,
                          g_cfg,
                          heading_deg,
                          fwd_in,
                          side_in,
                          guess_x,
                          guess_y);
}

} // namespace snapshot_pose
