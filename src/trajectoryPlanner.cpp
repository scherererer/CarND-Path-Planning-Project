
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
	std::vector<double> p0 = getXY(carState.s + deltaS, carState.d, worldModel_.map ());
	std::vector<double> p1 = getXY(carState.s + 2 * deltaS, carState.d, worldModel_.map ());
	std::vector<double> p2 = getXY(carState.s + 3 * deltaS, carState.d, worldModel_.map ());

	Trajectory t;

	t.x.push_back (p0[0]);
	t.y.push_back (p0[1]);

	t.x.push_back (p1[0]);
	t.y.push_back (p1[1]);

	t.x.push_back (p2[0]);
	t.y.push_back (p2[1]);
}
