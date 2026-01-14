#pragma once
#include "snapshot_pose/snapshot_pose.hpp"
#include "snapshot_pose/raycast.hpp"
#include <algorithm>
#include <cmath>


// Put this in the include/snapshot_pose folder

namespace snapshot_pose {

static inline float mm_to_in(float mm) { return mm / 25.4f; }

static float sigma_in_from_meas(float z_in) {
  // VEX spec: below 200mm (~7.874"), ±15mm (~0.5906");
  // above 200mm, ~5% of distance. :contentReference[oaicite:3]{index=3}
  if (z_in <= 7.874f) return 0.5906f;
  return 0.05f * z_in;
}

static bool read_distance_median_in(
  const DistanceSensorConfig& sc,
  const SnapshotConfig& cfg,
  float& out_z_in
) {
  std::vector<float> vals;
  vals.reserve(cfg.samples);

  for (int i = 0; i < cfg.samples; ++i) {
    const int mm = sc.dev->get();               // PROS distance returns mm
    const float z_in = mm_to_in((float)mm);

    // Confidence behavior is platform-dependent; PROS also exposes confidence (0..63).
    // Only gate confidence when distance > 200mm, because below 200mm many APIs clamp confidence. :contentReference[oaicite:4]{index=4}
    bool ok = (z_in >= cfg.min_range_in && z_in <= cfg.max_range_in);

    if (ok && sc.use_confidence_gate) {
      const int conf = sc.dev->get_confidence();
      if (z_in > 7.874f && conf < sc.min_confidence) ok = false;
    }

    if (ok) vals.push_back(z_in);
    pros::delay(cfg.sample_delay_ms);
  }

  if (vals.size() < (std::size_t)std::max(3, cfg.samples / 2)) return false;

  std::sort(vals.begin(), vals.end());
  out_z_in = vals[vals.size() / 2];
  return true;
}

static Vec2 rotate_robot_to_field_offset(float heading_deg_jar, float x_right, float y_fwd) {
  // For JAR heading:
  // forward unit f = (sin(th), cos(th))
  // right   unit r = (cos(th), -sin(th))
  const float th = heading_deg_jar * 3.1415926535f / 180.0f;
  const float s = std::sin(th);
  const float c = std::cos(th);

  const Vec2 f{ s,  c};
  const Vec2 r{ c, -s};

  return add(mul(r, x_right), mul(f, y_fwd));
}

static bool closest_points_between_segments(
  const Vec2& a0, const Vec2& a1,
  const Vec2& b0, const Vec2& b1,
  Vec2& out_pa, Vec2& out_pb
) {
  // If segments intersect, return intersection as both points.
  // Otherwise return closest endpoint-to-segment projections (sufficient in 2D).
  const Vec2 r = sub(a1, a0);
  const Vec2 s = sub(b1, b0);
  const float rxs = cross(r, s);
  const float q_pxr = cross(sub(b0, a0), r);
  const float eps = 1e-8f;

  if (std::fabs(rxs) > eps) {
    const float t = cross(sub(b0, a0), s) / rxs;
    const float u = cross(sub(b0, a0), r) / rxs;
    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
      const Vec2 p = add(a0, mul(r, t));
      out_pa = p;
      out_pb = p;
      return true;
    }
  } else {
    // parallel: no intersection handling needed for our use; fall through to closest.
    (void)q_pxr;
  }

  auto closest_on_seg = [](const Vec2& p, const Vec2& s0, const Vec2& s1) {
    const Vec2 d = sub(s1, s0);
    const float dd = dot(d, d);
    if (dd < 1e-9f) return s0;
    float t = dot(sub(p, s0), d) / dd;
    t = std::max(0.0f, std::min(1.0f, t));
    return add(s0, mul(d, t));
  };

  Vec2 best_pa = a0;
  Vec2 best_pb = closest_on_seg(a0, b0, b1);
  float best_d2 = dot(sub(best_pa, best_pb), sub(best_pa, best_pb));

  auto try_pair = [&](const Vec2& pa, const Vec2& pb) {
    const float d2 = dot(sub(pa, pb), sub(pa, pb));
    if (d2 < best_d2) { best_d2 = d2; best_pa = pa; best_pb = pb; }
  };

  // Endpoints of A to B
  try_pair(a0, closest_on_seg(a0, b0, b1));
  try_pair(a1, closest_on_seg(a1, b0, b1));
  // Endpoints of B to A
  try_pair(closest_on_seg(b0, a0, a1), b0);
  try_pair(closest_on_seg(b1, a0, a1), b1);

  out_pa = best_pa;
  out_pb = best_pb;
  return true;
}

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
) {
  SnapshotResult res;
  res.heading_deg = heading_deg_jar;

  if (sensors.size() < 2) return res;

struct Meas {
  float z_in;
    Vec2 o_field;
    Vec2 d_field;
    std::uint32_t mask;        // NEW: the mask used for this sensor
    std::vector<RayHit> hits;
  };


  std::vector<Meas> meas;
  meas.reserve(sensors.size());

  // Acquire median-filtered measurements and precompute offsets/dirs.
  for (const auto& sc : sensors) {
    float z_in = 0.0f;
    if (!read_distance_median_in(sc, cfg, z_in)) continue;

    // Sensor origin offset in field coords depends on heading.
    const Vec2 o_field = rotate_robot_to_field_offset(heading_deg_jar, sc.x_right_in, sc.y_fwd_in);

    // Sensor direction = heading + relative sensor direction (both JAR-style).
    const float dir_deg = heading_deg_jar + sc.rel_deg;
    const Vec2 d_field = unit_from_jar_deg(dir_deg);

    // Collect candidate hits using odom guess.
    const Vec2 guess_center{odom_guess_x_in, odom_guess_y_in};
    const Vec2 origin_guess = add(guess_center, o_field);

    const std::uint32_t mask_i = (sc.field_mask_override != 0u) ? sc.field_mask_override : cfg.field_mask;
    auto hits = raycast_all(origin_guess, d_field, cfg.max_range_in, mask_i);

    if (hits.empty()) continue;

    if ((int)hits.size() > cfg.candidates_per_sensor) hits.resize((std::size_t)cfg.candidates_per_sensor);

    meas.push_back(Meas{z_in, o_field, d_field, mask_i, hits});

  }

  if (meas.size() < 2) return res;

  // Enumerate combinations of segment choices (up to K^N; keep N small: 2-3 sensors recommended).
  const int N = (int)meas.size();
  const int K = std::max(1, cfg.candidates_per_sensor);

  auto idx_to_choice = [&](int code, std::vector<int>& choice) {
    choice.assign(N, 0);
    for (int i = 0; i < N; ++i) {
      const int ki = (int)meas[i].hits.size();
      const int base = std::max(1, std::min(K, ki));
      choice[i] = code % base;
      code /= base;
    }
  };

  // Compute how many combos (bounded; intended for N<=3).
  int combos = 1;
  for (int i = 0; i < N; ++i) {
    combos *= std::max(1, std::min(K, (int)meas[i].hits.size()));
    if (combos > 200) { combos = 200; break; } // hard cap
  }

  bool found = false;
  float best_chi2 = 1e30f;
  Vec2 best_pose{0,0};

  std::vector<int> choice;

  for (int code = 0; code < combos; ++code) {
    idx_to_choice(code, choice);

    // Build “robot-center locus segment” for each sensor:
    // If obstacle segment is A->B, then possible robot centers are:
    // X(u) = (A + u(B-A)) - o_field - d_field*z, u in [0,1]
    std::vector<Vec2> L0, L1;
    L0.reserve(N); L1.reserve(N);

    for (int i = 0; i < N; ++i) {
      const auto& hit = meas[i].hits[(std::size_t)choice[i]];
      const auto& seg = TERMINAL_FIELD_SEGMENTS[hit.seg_idx];

      const Vec2 shift = add(meas[i].o_field, mul(meas[i].d_field, meas[i].z_in));
      const Vec2 a = sub(seg.a, shift);
      const Vec2 b = sub(seg.b, shift);

      L0.push_back(a);
      L1.push_back(b);
    }

    // Estimate pose from pairwise closest-points midpoints, then robust aggregate.
    std::vector<Vec2> mids;
    for (int i = 0; i < N; ++i) {
      for (int j = i + 1; j < N; ++j) {
        Vec2 pa, pb;
        closest_points_between_segments(L0[i], L1[i], L0[j], L1[j], pa, pb);
        mids.push_back(mul(add(pa, pb), 0.5f));
      }
    }

    if (mids.empty()) continue;

    // Component-wise median of midpoints (robust).
    std::vector<float> xs, ys;
    xs.reserve(mids.size()); ys.reserve(mids.size());
    for (const auto& m : mids) { xs.push_back(m.x); ys.push_back(m.y); }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    Vec2 pose{ xs[xs.size()/2], ys[ys.size()/2] };

    // Validate by re-raycasting from this pose and scoring residuals.
    float chi2 = 0.0f;
    bool ok = true;

    for (int i = 0; i < N; ++i) {
      const Vec2 origin = add(pose, meas[i].o_field);
      const auto hits = raycast_all(origin, meas[i].d_field, cfg.max_range_in, meas[i].mask);
      if (hits.empty()) { ok = false; break; }

      const float z_pred = hits[0].t_in;
      const float z_meas = meas[i].z_in;

      const float sig = std::max(0.10f, sigma_in_from_meas(z_meas));
      const float e = (z_meas - z_pred) / sig;
      chi2 += e * e;

      if (e * e > cfg.max_chi2_per_sensor) { ok = false; break; }
    }

    if (!ok) continue;

    if (chi2 < best_chi2) {
      best_chi2 = chi2;
      best_pose = pose;
      found = true;
    }
  }

  if (!found) return res;

  // Clamp to field bounds (small margin)
  best_pose.x = std::max(-2.0f, std::min(FIELD_SIZE_IN + 2.0f, best_pose.x));
  best_pose.y = std::max(-2.0f, std::min(FIELD_SIZE_IN + 2.0f, best_pose.y));

  // Apply to odometry
  odom.set_position(best_pose.x, best_pose.y, heading_deg_jar, forward_tracker_in, sideways_tracker_in);

  res.ok = true;
  res.x_in = best_pose.x;
  res.y_in = best_pose.y;
  res.used_sensors = (int)meas.size();
  res.chi2 = best_chi2;
  return res;
}

} // namespace snapshot_pose
