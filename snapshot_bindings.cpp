// src/snapshot_pose/snapshot_bindings.cpp
//
// Snapshot Pose “bindings” layer:
// - Keeps all configuration in ONE place.
// - Your auton should only call snapshot_pose::snapshot_setpose_quadrant(...).
//
// What this does:
// - Reads 2–3 Distance sensors (median-filtered).
// - Uses raycasts against the collision map segments to infer robot (x,y) on the field.
// - Applies odom.set_position(x, y, heading, forwardTracker, sidewaysTracker).
//
// Reliability defaults:
// - Perimeter-only by default (walls are the most stable landmarks).
// - Per-sensor masks supported: low sensors can ignore interior objects that they can’t see.
//
// Coordinate system (must match your JAR odom):
// - Field is 144" x 144".
// - (0,0) bottom-left, +X right, +Y up.
// - heading_deg is JAR convention: 0° = +Y, clockwise-positive. :contentReference[oaicite:3]{index=3}

#include "snapshot_pose/snapshot_bindings.hpp"

#include <pros/rtos.hpp>
#include <pros/lcd.hpp>

// EDIT: include the header where your JAR odom instance is declared/defined.
#include "main.h"   // replace if your project uses a different global header

namespace snapshot_pose {

// -------------------------------
// EDIT SECTION 1: ODOM ACCESS
// -------------------------------
//
// You must have a global or accessible odom object with:
//   - float X_position, Y_position, orientation_deg
//   - void set_position(float x, float y, float heading_deg, float fwdTrackerIn, float sideTrackerIn)
//     (documented by JAR) :contentReference[oaicite:4]{index=4}
//
// If your odom is in a namespace or singleton, update the references below.

extern decltype(odom) odom; // If this fails, replace with your exact odom type and symbol name.

// -------------------------------
// EDIT SECTION 2: DISTANCE SENSORS
// -------------------------------
//
// If you already define these sensors elsewhere, replace these with `extern`
// and include the correct header above.
//
// Example:
//   extern pros::Distance distFront;
//   extern pros::Distance distRight;

static pros::Distance distFront(1); // EDIT: port
static pros::Distance distRight(2); // EDIT: port
// static pros::Distance distLeft(3); // optional third sensor

// -------------------------------
// EDIT SECTION 3: TRACKERS + HEADING SOURCES
// -------------------------------
//
// Snapshot pose assumes heading is correct and solves translation (x,y).
// Prefer to use the SAME heading variable your odom update uses.
//
// Trackers: set_position() takes current tracker distances (inches). :contentReference[oaicite:5]{index=5}
// If you have tracking wheels, provide the current distances in inches.
// If you do NOT have tracking wheels, you can return 0.0f for both.

static float get_heading_jar_deg() {
  // Preferred: reuse odom's heading (already in JAR convention)
  return odom.orientation_deg;
}

static float get_forward_tracker_in() {
  // EDIT:
  // return forwardTracker.get_position_in();  // your function
  return 0.0f;
}

static float get_sideways_tracker_in() {
  // EDIT:
  // return sidewaysTracker.get_position_in(); // your function
  return 0.0f;
}

// -------------------------------
// INTERNAL: configuration + sensor list
// -------------------------------
static SnapshotConfig g_cfg;
static std::vector<DistanceSensorConfig> g_sensors;
static bool g_init = false;

static void init_once() {
  if (g_init) return;
  g_init = true;

  // -------- Global snapshot config (safe defaults) --------
  g_cfg.field_mask = MAP_PERIMETER;        // default fallback if per-sensor override is 0
  g_cfg.candidates_per_sensor = 1;         // perimeter-only: 1 candidate is usually enough
  g_cfg.samples = 5;                       // median of 5 readings
  g_cfg.sample_delay_ms = 35;
  g_cfg.max_chi2_per_sensor = 9.0f;        // ~3-sigma per sensor
  g_cfg.quadrant_margin_in = 2.0f;

  // If you enable interior objects globally, increase candidates:
  // g_cfg.field_mask = MAP_PERIMETER | MAP_LONG_GOALS | MAP_CENTER_GOALS;
  // g_cfg.candidates_per_sensor = 2;

  // -------- Sensor definitions --------
  //
  // Units:
  //   x_right_in: inches from robot center, + to robot-right
  //   y_fwd_in:   inches from robot center, + to robot-forward
  //
  // rel_deg (relative to robot forward):
  //   0   = forward
  //   +90 = right
  //   180 = back
  //   -90 = left
  //
  // field_mask_override:
  //   0          => use g_cfg.field_mask
  //   nonzero    => use this mask for THIS sensor

  g_sensors.clear();
  g_sensors.reserve(3);

  // FRONT sensor (often low on the robot)
  {
    DistanceSensorConfig s{};
    s.dev = &distFront;

    // EDIT: measure these offsets
    s.x_right_in = 0.0f;
    s.y_fwd_in   = 7.0f;
    s.rel_deg    = 0.0f;

    // Per-sensor mask example:
    // Low-mounted sensor: only trust walls.
    s.field_mask_override = MAP_PERIMETER;

    // Confidence gating (0..63) only meaningful when distance > 200mm. :contentReference[oaicite:6]{index=6}
    s.use_confidence_gate = true;
    s.min_confidence = 35;

    // Optional gates (off by default):
    s.use_velocity_gate = false;
    s.use_object_size_gate = false;

    g_sensors.push_back(s);
  }

  // RIGHT sensor (maybe higher / different height)
  {
    DistanceSensorConfig s{};
    s.dev = &distRight;

    // EDIT: measure these offsets
    s.x_right_in = 7.0f;
    s.y_fwd_in   = 0.0f;
    s.rel_deg    = 90.0f;

    // If this sensor can “see” taller interior objects reliably, enable them here.
    // Example: walls + long goals only
    s.field_mask_override = MAP_PERIMETER | MAP_LONG_GOALS;

    s.use_confidence_gate = true;
    s.min_confidence = 35;

    s.use_velocity_gate = false;
    s.use_object_size_gate = false;

    g_sensors.push_back(s);
  }

  // OPTIONAL third sensor:
  // - Helpful if you sometimes cannot see two perpendicular walls due to range limits.
  //
  // {
  //   DistanceSensorConfig s{};
  //   s.dev = &distLeft;
  //   s.x_right_in = -7.0f;
  //   s.y_fwd_in = 0.0f;
  //   s.rel_deg = -90.0f;
  //   s.field_mask_override = MAP_PERIMETER;
  //   g_sensors.push_back(s);
  // }

  pros::lcd::print(6, "SnapshotPose init OK");
}

// -------------------------------
// Public API called by auton
// -------------------------------
SnapshotResult snapshot_setpose_quadrant(Quadrant q) {
  init_once();

  // Heading and tracker readings (inches) must match your odom usage. :contentReference[oaicite:7]{index=7}
  const float heading_deg = get_heading_jar_deg();
  const float fwd_in = get_forward_tracker_in();
  const float side_in = get_sideways_tracker_in();

  // Guess pose is used only to choose candidate segments for each sensor.
  // If you pass a quadrant, the solver also biases the guess toward that quadrant internally.
  const float guess_x = odom.X_position;
  const float guess_y = odom.Y_position;

  // Strongly recommended: call this while the robot is stopped.
  // (Do drive.brake(); delay(150-250ms); in auton before calling.)

  return snapshot_setpose(
    odom,
    g_sensors,
    g_cfg,
    heading_deg,
    fwd_in,
    side_in,
    guess_x,
    guess_y,
    q
  );
}

} // namespace snapshot_pose
