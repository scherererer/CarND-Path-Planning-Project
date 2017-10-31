
#pragma once

#include "coordXY.h"
#include "map.h"

#include <vector>

struct Trajectory
{
	std::vector<double> x;
	std::vector<double> y;
};

struct Candidate
{
	enum Mode
	{
		TARGET_SPEED,
		TARGET_ACCEL,
	};

	static Candidate generate (Trajectory const &seedTrajectory,
	                           Map const &map,
	                           CoordXY const &car_pos,
	                           CoordXY const &seed_end, double angle,
	                           double current_s, double current_d,
	                           double current_speed,
	                           double desired_s, double desired_d,
	                           Mode mode, double target);

	Trajectory trajectory;
	bool isSafe;
};

