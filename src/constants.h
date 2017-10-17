
#pragma once

/// \brief Number of lanes
unsigned constexpr NUM_LANES = 3;
/// \brief Width of lanes in meters
double constexpr LANE_WIDTH = 4;
/// \brief Speed limit in m/s
///
/// Actual is 22.352 but we round down for tolerances
double constexpr SPEED_LIMIT = 22;
/// \brief Speed limit squared
double constexpr SPEED_LIMIT_2 = SPEED_LIMIT * SPEED_LIMIT;
/// \brief Acceleration limit (m/s^2)
double constexpr ACCEL_LIMIT = 10;
double constexpr ACCEL_LIMIT_2 = ACCEL_LIMIT*ACCEL_LIMIT;
/// \brief Jerk limit (m/s^3)
double constexpr JERK_LIMIT = 10;
double constexpr JERK_LIMIT_2 = JERK_LIMIT*JERK_LIMIT;
/// \brief Time step in seconds
double constexpr TIME_STEP = 0.020;
