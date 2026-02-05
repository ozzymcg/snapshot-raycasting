//
// SNAPSHOT POSE: FULL SETUP + USAGE WALKTHROUGH (PROS, library-agnostic)
//
// IMPORTANT:
//   - If your project already has initialize()/autonomous()/opcontrol(), DO NOT keep duplicates.
//     Copy the “Usage” blocks into your existing functions.
//   - All configuration is meant to live in ONE place: src/snapshot_pose/snapshot_bindings.cpp
//     Your auton should only call snapshot_pose::snapshot_setpose_quadrant(...).
//
// REFERENCES:
//   PROS Distance: get() returns mm, 9999 if no object; get_confidence 0..63 meaningful when >200mm.

#include "main.h"
#include "snapshot_pose/snapshot_bindings.hpp"

namespace {

// Runtime callback stubs. Replace these with your odometry library calls.
//
// LemLib examples:
//   static float cb_heading(void*) { return chassis.getPose(false, false).theta; }
//   static float cb_guess_x(void*) { return chassis.getPose(false, false).x; }
//   static float cb_guess_y(void*) { return chassis.getPose(false, false).y; }
//   static void cb_apply_pose(void*, float x, float y, float h, float, float) { chassis.setPose(x, y, h); }
//
// JAR examples:
//   static float cb_heading(void*) { return odom.orientation_deg; }
//   static float cb_guess_x(void*) { return odom.X_position; }
//   static float cb_guess_y(void*) { return odom.Y_position; }
//   static void cb_apply_pose(void*, float x, float y, float h, float f, float s) { odom.set_position(x, y, h, f, s); }
static float cb_heading(void*) { return 0.0f; }
static float cb_guess_x(void*) { return 0.0f; }
static float cb_guess_y(void*) { return 0.0f; }
static float cb_tracker_fwd(void*) { return 0.0f; }
static float cb_tracker_side(void*) { return 0.0f; }
static void cb_apply_pose(void*, float, float, float, float, float) {}

// ------------------------------
// STEP 0: Sanity notes (read once)
// ------------------------------
//
// Coordinate assumptions used by snapshot pose:
//   Field: 144" x 144"
//   Origin (0,0): bottom-left
//   +X right/forward off wall, +Y up/towards left matchloader
//
// Heading convention assumed by solver:
//   heading_deg: 0° points +Y, clockwise-positive. 
//
// Quadrants are defined by split lines x=72, y=72 (inches):
//   BL: x in [0,72],   y in [0,72]
//   BR: x in [72,144], y in [0,72]
//   TL: x in [0,72],   y in [72,144]
//   TR: x in [72,144], y in [72,144]
//
// Snapshot pose should be called ONLY when:
//   - drivetrain is stopped (braked)
//   - robot is not touching random objects
//   - sensors can see stable geometry (usually walls)
//

// ------------------------------
// STEP 1: Required project files (check list)
// ------------------------------
//
// You should have these files already added:
//
//   include/snapshot_pose/collision_map.hpp
//   include/snapshot_pose/raycast.hpp
//   include/snapshot_pose/snapshot_pose.hpp
//   include/snapshot_pose/snapshot_pose_impl.hpp
//   include/snapshot_pose/snapshot_bindings.hpp
//
//   src/snapshot_pose/raycast.cpp
//   src/snapshot_pose/snapshot_bindings.cpp
//
// If any are missing, snapshot_bindings.hpp will not compile.

// ------------------------------
// STEP 2: Configuration location (ONE file)
// ------------------------------
//
// All configuration is in:
//   src/snapshot_pose/snapshot_bindings.cpp
//
// What the user edits there:
//   - Distance sensor ports (or extern your existing sensors)
//   - Sensor offsets from robot center (x_right_in, y_fwd_in) in inches
//   - Sensor facing direction (rel_deg)
//   - Per-sensor field masks (field_mask_override)
//   - Hook functions for heading + tracker distances
//
// After that, usage in auton is one line:
//   snapshot_pose::snapshot_setpose_quadrant(...)

} // namespace

// ------------------------------
// STEP 3: Usage examples
// ------------------------------

void initialize() {
  pros::lcd::initialize();
  pros::lcd::print(0, "Snapshot Pose Example");

  snapshot_pose::SnapshotPoseRuntime runtime{};
  runtime.get_heading_deg = cb_heading;
  runtime.get_guess_x_in = cb_guess_x;
  runtime.get_guess_y_in = cb_guess_y;
  runtime.get_forward_tracker_in = cb_tracker_fwd;
  runtime.get_sideways_tracker_in = cb_tracker_side;
  runtime.apply_pose = cb_apply_pose;

  if (!snapshot_pose::snapshot_bindings_set_runtime(runtime)) {
    pros::lcd::print(1, "Snapshot runtime setup failed");
  }

  // (Optional) If you want to guide:
  pros::lcd::print(2, "Configure bindings + runtime");
  pros::lcd::print(3, "Then call snapshot_setpose_quadrant()");
}

void autonomous() {
  // ------------------------------
  // STEP 3A: Ensure the robot is in a good state to snapshot
  // ------------------------------
  //
  // 1) Stop / brake the drivetrain.
  //    Replace these with your drive API.
  //
  // Example:
  //   drive.brake();
  //   drive.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  //
  // The goal is “robot not moving / rotating”.

  pros::delay(200); // allow motion to settle

  // ------------------------------
  // STEP 3B: Choose quadrant (recommended)
  // ------------------------------
  //
  // If you KNOW the robot is in a quadrant, pass it.
  // This prevents wrong snaps if multiple solutions exist.
  //
  // Quadrant labels:
  //   BL  = Bottom-Left
  //   BR  = Bottom-Right
  //   TL  = Top-Left
  //   TR  = Top-Right
  //   ANY = No quadrant restriction
  //
  // Example: at the start of many routines you know your starting tile region.

  snapshot_pose::Quadrant q = snapshot_pose::Quadrant::TR;  //EDIT to match your known region

  // If you truly do not know:
  // snapshot_pose::Quadrant q = snapshot_pose::Quadrant::ANY;

  // ------------------------------
  // STEP 3C: Call snapshot pose (one line)
  // ------------------------------
  //
  // What happens internally:
  //   - Reads each configured Distance sensor several times (median)
  //   - Raycasts into collision map segments using per-sensor masks
  //   - Solves for (x,y) consistent across sensors
  //   - Validates residual errors
  //   - If successful: calls odom.set_position(...) 
  //   - If fail: DOES NOT modify pose

  const auto r = snapshot_pose::snapshot_setpose_quadrant(q);

  // ------------------------------
  // STEP 3D: Handle the result
  // ------------------------------
  //
  // Recommended pattern:
  //   - If OK: continue routine
  //   - If FAIL: continue using existing odom (do not “force” a pose)
  //
  // The call is designed to be safe: failure means no pose change.

  if (r.ok) {
    pros::lcd::print(3, "SNAP OK");
    pros::lcd::print(4, "x=%.2f y=%.2f", r.x_in, r.y_in);
    pros::lcd::print(5, "used=%d chi2=%.2f", r.used_sensors, r.chi2);
  } else {
    pros::lcd::print(3, "SNAP FAIL");
    pros::lcd::print(4, "Pose unchanged");
  }

  // Continue your auton...
  // drive.moveToPoint(...);
}

void opcontrol() {
  // ------------------------------
  // STEP 4: Driver-triggered debug usage (optional)
  // ------------------------------
  //
  // This is useful to confirm your sensor offsets and masks are correct.
  // You can park the robot in known spots and press a button to snap.

  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      // 1) Stop drive (edit with your API)
      // drive.brake();
      pros::delay(200);

      // 2) Snap without quadrant restriction (or choose one)
      const auto r = snapshot_pose::snapshot_setpose_quadrant(snapshot_pose::Quadrant::ANY);

      // 3) Display
      if (r.ok) {
        pros::lcd::print(6, "DBG OK x=%.1f y=%.1f", r.x_in, r.y_in);
      } else {
        pros::lcd::print(6, "DBG FAIL");
      }
    }

    pros::delay(10);
  }
}

/* --------------------------------------------------------------------------
   STEP 5: Library-specific setup points
   --------------------------------------------------------------------------

Configure runtime callbacks in initialize() (this file):
  runtime.get_heading_deg
  runtime.get_guess_x_in / runtime.get_guess_y_in
  runtime.apply_pose
  runtime.get_forward_tracker_in / runtime.get_sideways_tracker_in (optional)

LemLib callback example:
  runtime.get_heading_deg = [](void*) { return chassis.getPose(false, false).theta; };
  runtime.get_guess_x_in  = [](void*) { return chassis.getPose(false, false).x; };
  runtime.get_guess_y_in  = [](void*) { return chassis.getPose(false, false).y; };
  runtime.apply_pose      = [](void*, float x, float y, float h, float, float) { chassis.setPose(x, y, h); };

JAR callback example:
  runtime.get_heading_deg = [](void*) { return odom.orientation_deg; };
  runtime.get_guess_x_in  = [](void*) { return odom.X_position; };
  runtime.get_guess_y_in  = [](void*) { return odom.Y_position; };
  runtime.apply_pose      = [](void*, float x, float y, float h, float f, float s) { odom.set_position(x, y, h, f, s); };

Configure sensor geometry in:
  src/snapshot_pose/snapshot_bindings.cpp

  - distance sensor ports
  - x_right_in / y_fwd_in / rel_deg (mount pose per sensor)
  - field masks (per-sensor or global)
  - sample count and gating knobs

Map mask quick reference (collision_map.hpp):
  MAP_PERIMETER
  MAP_LONG_GOALS
  MAP_LONG_GOAL_BRACES
  MAP_LONG_GOALS_ALL
  MAP_CENTER_GOAL_POS45 / MAP_CENTER_GOAL_NEG45 / MAP_CENTER_GOALS
  MAP_MATCHLOADERS
  MAP_PARK_ZONES

Per-sensor collision examples:
  // Front sensor: only walls
  front.field_mask_override = MAP_PERIMETER;

  // Right sensor: walls + long goals + braces
  right.field_mask_override = MAP_PERIMETER | MAP_LONG_GOALS_ALL;

Runtime sensor customization (without editing defaults):
  snapshot_pose::DistanceSensorConfig s = {};
  s.dev = &distFront;
  s.x_right_in = 0.0f;
  s.y_fwd_in = 7.0f;
  s.rel_deg = 0.0f;
  s.field_mask_override = MAP_PERIMETER;
  snapshot_pose::snapshot_bindings_update_sensor(0, s);
  snapshot_pose::snapshot_bindings_set_sensor_mask(1, MAP_PERIMETER | MAP_LONG_GOALS_ALL);

--------------------------------------------------------------------------- */
