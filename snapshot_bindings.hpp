#pragma once
#include "snapshot_pose/snapshot_pose.hpp"

namespace snapshot_pose {

// One-call API for auton.
// If ok==false, pose is not modified.
// If ok==true, pose is applied via odom.set_position(...).
SnapshotResult snapshot_setpose_quadrant(Quadrant q = Quadrant::ANY);

} // namespace snapshot_pose
