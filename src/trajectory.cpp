
#include "trajectory.h"
#include "constants.h"
#include "utilities.h"
#include "worldModel.h"

#include "tk/spline.h"

#include <cmath>

namespace
{

int constexpr STEP_HORIZON = 100;

double constexpr MIN_MANHATTAN_DISTANCE_FOR_START_WAYPOINT = 0.001;

}

Candidate Candidate::generate (
	Trajectory const &seedTrajectory, Map const &map, CoordXY const &car_pos,
	CoordXY const &seed_end, double angle,
	double current_s, double current_d,
	double current_speed,
	double desired_s, double desired_d,
	Mode mode, double target)
{
	Candidate c;

	c.trajectory = seedTrajectory;
	size_t const previous_path_size = seedTrajectory.x.size ();

	double s = current_s;
	double d = current_d;

	std::vector<double> splineX;
	std::vector<double> splineY;

	// Add the car's XY
	if (previous_path_size > 0)
	{
		double const shift_x = car_pos.x() - seed_end.x();
		double const shift_y = car_pos.y() - seed_end.y();

		// Check the manhattan distance is greater than the threshold, otherwise don't bother
		// adding this point.
		if (shift_x + shift_y > MIN_MANHATTAN_DISTANCE_FOR_START_WAYPOINT)
		{
			splineX.push_back(shift_x * cos(0 - angle) - shift_y * sin(0 - angle));
			splineY.push_back(shift_x * sin(0 - angle) + shift_y * cos(0 - angle));
		}
	}

	// First position, needs to be added without introducing error of xy->frenet->xy
	splineX.push_back(0);
	splineY.push_back(0);

	for (unsigned i = 1; i < 6; ++i)
	{
		// Spacing of 20m is set here, speed is handled later
		/// \todo Tweak spacing for candidate generation?
		s = advanceS (s, 15, map);
		d = ramp (d, desired_d, 1.0);

		std::vector<double> const xy = getXY(s, d, map);

		double const shift_x = xy[0] - seed_end.x();
		double const shift_y = xy[1] - seed_end.y();

		splineX.push_back(shift_x * cos(0 - angle) - shift_y * sin(0 - angle));
		splineY.push_back(shift_x * sin(0 - angle) + shift_y * cos(0 - angle));
	}

	tk::spline spline;

	spline.set_points (splineX, splineY);

	//double constexpr ACCEL = 0.1;
	double constexpr ACCEL = 0.05;

	double x = 0;
	double v = current_speed;

	for (int i = 1; i < STEP_HORIZON - previous_path_size; ++i)
	{
		// x, y in vehicle space
		double const y = spline (x);

		switch (mode)
		{
		case TARGET_SPEED:
			x = x + v * TIME_STEP;
			v = ramp(v, target, ACCEL);
			break;
		case TARGET_ACCEL:
			x = x + v * TIME_STEP + 0.5 * target * TIME_STEP * TIME_STEP;
			//v += target * TIME_STEP;
			break;
		}

		// now convert to world space
		double const wx = (x * cos(angle) - y * sin(angle)) + seed_end.x();
		double const wy = (x * sin(angle) + y * cos(angle)) + seed_end.y();

		//assert (c.trajectory.x.empty () || (fabs(wx - c.trajectory.x.back()) < SPEED_LIMIT));

		c.trajectory.x.push_back(wx);
		c.trajectory.y.push_back(wy);
	}

	return c;
}
