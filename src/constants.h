
#pragma once

/// \brief Number of lanes
unsigned constexpr NUM_LANES = 3;
/// \brief Width of lanes in meters
double constexpr LANE_WIDTH = 4;
/// \brief Speed limit in m/s
///
/// Actual is 22.352 but we round down for tolerances
double constexpr SPEED_LIMIT = 22;
/// \brief Time step in seconds
double constexpr TIME_STEP = 0.020;
