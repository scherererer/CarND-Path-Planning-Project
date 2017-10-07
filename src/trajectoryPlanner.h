
#pragma once

#include "carState.h"
#include "maneuver.h"
#include "worldModel.h"

#include <vector>

class TrajectoryPlanner
{
public:
	struct Trajectory
	{
		std::vector<double> x;
		std::vector<double> y;
	};

	~TrajectoryPlanner ();
	explicit TrajectoryPlanner (WorldModel const &worldModel);

	Trajectory update (CarState const &carState, Maneuver const &desiredManeuver);

private:
	/// \brief World model
	WorldModel const &worldModel_;
};
