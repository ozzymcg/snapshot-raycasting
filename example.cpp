//
// SNAPSHOT POSE: FULL SETUP + USAGE WALKTHROUGH (PROS + JAR)
//
// IMPORTANT:
//   - If your project already has initialize()/autonomous()/opcontrol(), DO NOT keep duplicates.
//     Copy the “Usage” blocks into your existing functions.
//   - All configuration is meant to live in ONE place: src/snapshot_pose/snapshot_bindings.cpp
//     Your auton should only call snapshot_pose::snapshot_setpose_quadrant(...).
//
// REFERENCES:
//   PROS Distance: get() returns mm, 9999 if no object; get_confidence 0..63 meaningful when >200mm. 
//   JAR odom set_position signature + orientation convention documented in JAR odometry docs. 

#include "main.h"
#include "snapshot_pose/snapshot_bindings.hpp"

namespace {

// ------------------------------
// STEP 0: Sanity notes (read once)
// ------------------------------
//
// Coordinate assumptions used by snapshot pose:
//   Field: 144" x 144"
//   Origin (0,0): bottom-left
//   +X right/forward off wall, +Y up/towards left matchloader
//
// JAR heading convention assumed by solver and by set_position call:
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

  // (Optional) If you want to guide users:
  pros::lcd::print(1, "Edit bindings.cpp to configure");
  pros::lcd::print(2, "Then call snapshot_setpose_quadrant()");
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
  // The goal is “robot not translating / rotating”.

  pros::delay(200); // allow motion to settle

  // ------------------------------
  // STEP 3B: Choose quadrant (recommended)
  // ------------------------------
  //
  // If you KNOW the robot is in a quadrant, pass it.
  // This prevents wrong snaps if multiple solutions exist.
  //
  // Example: at the start of many routines you know your starting tile region.

  snapshot_pose::Quadrant q = snapshot_pose::Quadrant::TR;  // EDIT to match your known region

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
   STEP 5: What the user must edit in src/snapshot_pose/snapshot_bindings.cpp
   --------------------------------------------------------------------------

Open:  src/snapshot_pose/snapshot_bindings.cpp

Edit these sections:

(1) ODOM ACCESS
    - Include the header that defines your odom object.
    - Ensure the file can reference:
        odom.X_position
        odom.Y_position
        odom.orientation_deg   (recommended heading source)
        odom.set_position(x,y,heading,fwd,side) 

(2) DISTANCE SENSOR PORTS
    - Either define sensors there:
        static pros::Distance distFront(PORT);
      or use extern to refer to your existing devices.

(3) TRACKERS + HEADING SOURCES
    - Provide get_heading_jar_deg()
        return odom.orientation_deg;   // best default
    - Provide get_forward_tracker_in() and get_sideways_tracker_in()
        return your tracking-wheel distances in inches (same values your odom update uses)
      If you don't have tracking wheels:
        return 0.0f; (and ensure your odom supports that)

(4) SENSOR MOUNT OFFSETS
    - Measure offsets in inches from robot center:
        x_right_in (positive to robot right)
        y_fwd_in   (positive to robot forward)
    - Set rel_deg:
        0 forward, +90 right, 180 back, -90 left

(5) PER-SENSOR FIELD MASKS (HEIGHT DIFFERENCES)
    - Example:
        s.field_mask_override = MAP_PERIMETER;
      for a low sensor that cannot see interior objects well.

    - Another sensor might allow more:
        s.field_mask_override = MAP_PERIMETER | MAP_LONG_GOALS | MAP_CENTER_GOALS;

    - Global fallback:
        g_cfg.field_mask = MAP_PERIMETER;  // used if override == 0

(6) CANDIDATES
    - If any sensor can see interior field objects, increase:
        g_cfg.candidates_per_sensor = 2;
      This lets the solver try multiple “hit explanations” per sensor.

After edits:
  - Build and run.
  - Use opcontrol debug button to test snap in known locations.
  - If snaps are wrong:
      * restrict masks (use perimeter only)
      * ensure offsets are correct
      * ensure robot is stopped before snapping
      * use a quadrant when possible

--------------------------------------------------------------------------- */
