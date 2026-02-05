#pragma once
#include <array>
#include <cstdint>

// Put this in the include/snapshot_pose folder

namespace snapshot_pose {

// Field is 144" x 144". Coordinate system:
// origin (0,0) = bottom-left, +X right, +Y up.
inline constexpr float FIELD_SIZE_IN = 144.0f;

struct Vec2 {
  float x;
  float y;
};

struct Segment {
  Vec2 a;
  Vec2 b;
  std::uint32_t mask;
};

enum MapMask : std::uint32_t {
  MAP_NONE               = 0u,
  MAP_PERIMETER          = 1u << 0,
  MAP_LONG_GOALS         = 1u << 1,
  MAP_LONG_GOAL_BRACES   = 1u << 6,

  // Center goals split into two independent masks:
  MAP_CENTER_GOAL_POS45  = 1u << 2,
  MAP_CENTER_GOAL_NEG45  = 1u << 3,

  MAP_MATCHLOADERS       = 1u << 4,
  MAP_PARK_ZONES         = 1u << 5,

  // Optional convenience aliases:
  MAP_LONG_GOALS_ALL     = (MAP_LONG_GOALS | MAP_LONG_GOAL_BRACES),
  MAP_CENTER_GOALS       = (MAP_CENTER_GOAL_POS45 | MAP_CENTER_GOAL_NEG45),

  MAP_ALL                = 0xFFFFFFFFu
};

// NOTE (reliability):
// Defaulting to perimeter-only avoids “false-short” hits on small objects.
// Add interior objects only if you are confident your sensors will actually see them.
inline constexpr std::uint32_t DEFAULT_FIELD_MASK = MAP_PERIMETER;

// Segments derived from Terminal geometry (draw.py field objects).
inline constexpr std::array<Segment, 60> TERMINAL_FIELD_SEGMENTS = {{
  Segment{Vec2{0.f, 0.f}, Vec2{144.f, 0.f}, MAP_PERIMETER},
  Segment{Vec2{144.f, 0.f}, Vec2{144.f, 144.f}, MAP_PERIMETER},
  Segment{Vec2{144.f, 144.f}, Vec2{0.f, 144.f}, MAP_PERIMETER},
  Segment{Vec2{0.f, 144.f}, Vec2{0.f, 0.f}, MAP_PERIMETER},

  Segment{Vec2{25.75f, 47.6f}, Vec2{25.75f, 96.4f}, MAP_LONG_GOALS},
  Segment{Vec2{25.75f, 96.4f}, Vec2{22.25f, 96.4f}, MAP_LONG_GOALS},
  Segment{Vec2{22.25f, 96.4f}, Vec2{22.25f, 47.6f}, MAP_LONG_GOALS},
  Segment{Vec2{22.25f, 47.6f}, Vec2{25.75f, 47.6f}, MAP_LONG_GOALS},

  Segment{Vec2{121.75f, 47.6f}, Vec2{121.75f, 96.4f}, MAP_LONG_GOALS},
  Segment{Vec2{121.75f, 96.4f}, Vec2{118.25f, 96.4f}, MAP_LONG_GOALS},
  Segment{Vec2{118.25f, 96.4f}, Vec2{118.25f, 47.6f}, MAP_LONG_GOALS},
  Segment{Vec2{118.25f, 47.6f}, Vec2{121.75f, 47.6f}, MAP_LONG_GOALS},

  // Long goal braces (matches Atticus Terminal field object map)
  Segment{Vec2{25.75f, 92.9f}, Vec2{25.75f, 96.4f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{25.75f, 96.4f}, Vec2{22.25f, 96.4f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{22.25f, 96.4f}, Vec2{22.25f, 92.9f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{22.25f, 92.9f}, Vec2{25.75f, 92.9f}, MAP_LONG_GOAL_BRACES},

  Segment{Vec2{25.75f, 47.6f}, Vec2{25.75f, 51.1f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{25.75f, 51.1f}, Vec2{22.25f, 51.1f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{22.25f, 51.1f}, Vec2{22.25f, 47.6f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{22.25f, 47.6f}, Vec2{25.75f, 47.6f}, MAP_LONG_GOAL_BRACES},

  Segment{Vec2{121.75f, 92.9f}, Vec2{121.75f, 96.4f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{121.75f, 96.4f}, Vec2{118.25f, 96.4f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{118.25f, 96.4f}, Vec2{118.25f, 92.9f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{118.25f, 92.9f}, Vec2{121.75f, 92.9f}, MAP_LONG_GOAL_BRACES},

  Segment{Vec2{121.75f, 47.6f}, Vec2{121.75f, 51.1f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{121.75f, 51.1f}, Vec2{118.25f, 51.1f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{118.25f, 51.1f}, Vec2{118.25f, 47.6f}, MAP_LONG_GOAL_BRACES},
  Segment{Vec2{118.25f, 47.6f}, Vec2{121.75f, 47.6f}, MAP_LONG_GOAL_BRACES},

  // Center Goal (+45)
  Segment{Vec2{65.247131f, 62.772255f}, Vec2{81.227745f, 78.752869f}, MAP_CENTER_GOAL_POS45},
  Segment{Vec2{81.227745f, 78.752869f}, Vec2{78.752869f, 81.227745f}, MAP_CENTER_GOAL_POS45},
  Segment{Vec2{78.752869f, 81.227745f}, Vec2{62.772255f, 65.247131f}, MAP_CENTER_GOAL_POS45},
  Segment{Vec2{62.772255f, 65.247131f}, Vec2{65.247131f, 62.772255f}, MAP_CENTER_GOAL_POS45},

  // Center Goal (-45)
  Segment{Vec2{62.772255f, 78.752869f}, Vec2{78.752869f, 62.772255f}, MAP_CENTER_GOAL_NEG45},
  Segment{Vec2{78.752869f, 62.772255f}, Vec2{81.227745f, 65.247131f}, MAP_CENTER_GOAL_NEG45},
  Segment{Vec2{81.227745f, 65.247131f}, Vec2{65.247131f, 81.227745f}, MAP_CENTER_GOAL_NEG45},
  Segment{Vec2{65.247131f, 81.227745f}, Vec2{62.772255f, 78.752869f}, MAP_CENTER_GOAL_NEG45},


  Segment{Vec2{26.5f, 0.f}, Vec2{26.5f, 5.f}, MAP_MATCHLOADERS},
  Segment{Vec2{26.5f, 5.f}, Vec2{21.5f, 5.f}, MAP_MATCHLOADERS},
  Segment{Vec2{21.5f, 5.f}, Vec2{21.5f, 0.f}, MAP_MATCHLOADERS},
  Segment{Vec2{21.5f, 0.f}, Vec2{26.5f, 0.f}, MAP_MATCHLOADERS},

  Segment{Vec2{26.5f, 139.f}, Vec2{26.5f, 144.f}, MAP_MATCHLOADERS},
  Segment{Vec2{26.5f, 144.f}, Vec2{21.5f, 144.f}, MAP_MATCHLOADERS},
  Segment{Vec2{21.5f, 144.f}, Vec2{21.5f, 139.f}, MAP_MATCHLOADERS},
  Segment{Vec2{21.5f, 139.f}, Vec2{26.5f, 139.f}, MAP_MATCHLOADERS},

  Segment{Vec2{122.5f, 0.f}, Vec2{122.5f, 5.f}, MAP_MATCHLOADERS},
  Segment{Vec2{122.5f, 5.f}, Vec2{117.5f, 5.f}, MAP_MATCHLOADERS},
  Segment{Vec2{117.5f, 5.f}, Vec2{117.5f, 0.f}, MAP_MATCHLOADERS},
  Segment{Vec2{117.5f, 0.f}, Vec2{122.5f, 0.f}, MAP_MATCHLOADERS},

  Segment{Vec2{122.5f, 139.f}, Vec2{122.5f, 144.f}, MAP_MATCHLOADERS},
  Segment{Vec2{122.5f, 144.f}, Vec2{117.5f, 144.f}, MAP_MATCHLOADERS},
  Segment{Vec2{117.5f, 144.f}, Vec2{117.5f, 139.f}, MAP_MATCHLOADERS},
  Segment{Vec2{117.5f, 139.f}, Vec2{122.5f, 139.f}, MAP_MATCHLOADERS},

  Segment{Vec2{62.565f, 0.f}, Vec2{81.435f, 0.f}, MAP_PARK_ZONES},
  Segment{Vec2{81.435f, 0.f}, Vec2{81.435f, 16.86f}, MAP_PARK_ZONES},
  Segment{Vec2{81.435f, 16.86f}, Vec2{62.565f, 16.86f}, MAP_PARK_ZONES},
  Segment{Vec2{62.565f, 16.86f}, Vec2{62.565f, 0.f}, MAP_PARK_ZONES},

  Segment{Vec2{62.565f, 127.14f}, Vec2{81.435f, 127.14f}, MAP_PARK_ZONES},
  Segment{Vec2{81.435f, 127.14f}, Vec2{81.435f, 144.f}, MAP_PARK_ZONES},
  Segment{Vec2{81.435f, 144.f}, Vec2{62.565f, 144.f}, MAP_PARK_ZONES},
  Segment{Vec2{62.565f, 144.f}, Vec2{62.565f, 127.14f}, MAP_PARK_ZONES},
}};

} // namespace snapshot_pose
