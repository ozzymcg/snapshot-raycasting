#pragma once
#include <cstdint>
#include <vector>
#include "snapshot_pose/collision_map.hpp"


// Put this in the include/snapshot_pose folder

namespace snapshot_pose {

struct RayHit {
  float t_in;          // distance along ray in inches (ray direction assumed unit)
  std::size_t seg_idx; // index into TERMINAL_FIELD_SEGMENTS
  Vec2 point;          // hit point
};

float cross(const Vec2& a, const Vec2& b);
Vec2  sub(const Vec2& a, const Vec2& b);
Vec2  add(const Vec2& a, const Vec2& b);
Vec2  mul(const Vec2& a, float s);
float dot(const Vec2& a, const Vec2& b);
float norm(const Vec2& a);
Vec2  unit_from_heading_deg(float heading_deg);

// Returns all hits (sorted nearest-first). If you only want the nearest K, take the first K.
std::vector<RayHit> raycast_all(
  const Vec2& origin,
  const Vec2& dir_unit,
  float max_range_in,
  std::uint32_t allow_mask
);

} // namespace snapshot_pose
