
#include "trajectoryPlanner.h"
#include "constants.h"
#include "utilities.h"


TrajectoryPlanner::~TrajectoryPlanner ()
{
}

TrajectoryPlanner::TrajectoryPlanner (WorldModel const &worldModel)
	: worldModel_ (worldModel)
{
}

TrajectoryPlanner::Trajectory TrajectoryPlanner::update (CarState const &carState,
														 Maneuver const &desiredManeuver)
{
	double const deltaS = desiredManeuver.targetSpeed_ * TIME_STEP;
	Trajectory t;

	for (int i = 0; i < 100; ++i)
	{
		std::vector<double> p0 = getXY(carState.s + (i * deltaS),
		                               carState.d, worldModel_.map ());

		t.x.push_back (p0[0]);
		t.y.push_back (p0[1]);
	}

	return t;
}
