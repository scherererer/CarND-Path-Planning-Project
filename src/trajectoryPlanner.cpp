
#include "trajectoryPlanner.h"
#include "constants.h"
#include "utilities.h"

#include "Eigen-3.3/Eigen/Dense"

#include "tk/spline.h"

#include <iostream>
#include <limits>


namespace
{

double constexpr defaultManeuverDistance = 100;
double constexpr S_MEAN = 0.0;
double constexpr S_STD_DEV = 20.0;

double constexpr defaultManeuverHorizon = 10;
double constexpr TIME_MEAN = 0.0;
double constexpr TIME_STD_DEV = 4.5;

double constexpr D_MEAN = 0.0;
double constexpr D_STD_DEV = LANE_WIDTH / 6.0;

double constexpr V_MIN = 1.0;
double constexpr V_MEAN = 0.0;
double constexpr V_STD_DEV = SPEED_LIMIT / 3.0;

int constexpr STEP_HORIZON = 100;

unsigned constexpr NUM_TRAJECTORIES = 1000;


/// \brief Calculate the Jerk Minimizing Trajectory that connects the initial state
/// to the final state in time T.
/// \param start The vehicles start location given as a length three array
/// corresponding to initial values of [s, s_dot, s_double_dot]
/// \param end the desired end state for vehicle. Like "start" this is a
/// length three array.
/// \param T The duration, in seconds, over which this maneuver should occur.
/// \returns an array of length 6, each value corresponding to a coefficent in the polynomial
/// s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
inline std::vector<double> JMT(std::vector<double> start, std::vector<double> end, double T)
{
	Eigen::MatrixXd C(3,3);
	double const T2 = T * T;
	double const T3 = T2 * T;
	double const T4 = T3 * T;
	double const T5 = T4 * T;

	C <<   T3,    T4,    T5,
	     3*T2,  4*T3,  5*T4,
	      6*T, 12*T2, 20*T3;

	Eigen::VectorXd s = Eigen::VectorXd::Zero(3);

	s << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T2),
	     end[1] - (start[1] + start[2]*T),
	     end[2] - start[2];

	Eigen::VectorXd a2 = Eigen::VectorXd::Zero(3);
	//if (fabs (C.determinant ()) > 0.0001)
	a2 = C.inverse () * s;

	return {start[0], start[1], 0.5*start[2], a2[0], a2[1], a2[2]};
}

/// \brief Evaluate the polynomial
inline double evalPoly (std::vector<double> const &coeffs, double const t)
{
	double x = 1.0;
	double y = 0;

	for (double c : coeffs)
	{
		y += x * c;
		x *= t;
	}

	return y;
}

}


TrajectoryPlanner::~TrajectoryPlanner ()
{
}

TrajectoryPlanner::TrajectoryPlanner (WorldModel const &worldModel)
	: worldModel_ (worldModel)
	, randDevice_ ()
	, randEngine_ (randDevice_ ())
	, sRand_ (S_MEAN, S_STD_DEV)
	, dRand_ (D_MEAN, D_STD_DEV)
	, timeRand_ (TIME_MEAN, TIME_STD_DEV)
	, vRand_ (V_MEAN, V_STD_DEV)
{
}

TrajectoryPlanner::Trajectory TrajectoryPlanner::update (
	std::vector<double> const &previous_path_x, std::vector<double> const &previous_path_y,
	double end_path_s, double end_path_d,
	CarState const &carState, Maneuver const &desiredManeuver)
{
	std::cout << "-----------------------------------\n"
	          << "Lane: " << getLane (carState.d) << " (" << carState.d  << ") -> "
	          << desiredManeuver.targetLaneId_
	          << " (" << laneToD (desiredManeuver.targetLaneId_) << ")\n"
	          << "Speed: " << carState.speed << " -> " << desiredManeuver.targetSpeed_ << "\n"
	          << "Target Vehicle: " << desiredManeuver.targetLeadingVehicleId_ << "\n"
	          << "ETA: " << desiredManeuver.secondsToReachTarget_ << "\n"
	          << std::endl;

	size_t const previous_path_size = std::min (previous_path_x.size (), size_t (5));

	Trajectory t;

	for (unsigned i = 0; i < previous_path_size; ++i)
	{
		t.x.push_back (previous_path_x[i]);
		t.y.push_back (previous_path_y[i]);
	}

	double pos_x = 0;
	double pos_y = 0;
	double angle = 0;
	double end_speed = 0;
	double end_accel = 0;

	if(previous_path_size < 3)
	{
		pos_x = carState.x;
		pos_y = carState.y;
		angle = carState.yaw;

		std::vector<double> end = getFrenet(pos_x, pos_y,
											angle, worldModel_.map ());
		end_path_s = end[0];
		end_path_d = end[1];
	}
	else
	{
		pos_x = previous_path_x[previous_path_size-1];
		pos_y = previous_path_y[previous_path_size-1];

		double pos_x2 = previous_path_x[previous_path_size-2];
		double pos_y2 = previous_path_y[previous_path_size-2];

		double const xd = pos_x-pos_x2;
		double const yd = pos_y-pos_y2;

		angle = atan2(yd, xd);
		end_speed = std::sqrt (xd * xd + yd * yd) / TIME_STEP;

		double pos_x3 = previous_path_x[previous_path_size-3];
		double pos_y3 = previous_path_y[previous_path_size-3];

		double const xd2 = pos_x2-pos_x3;
		double const yd2 = pos_y2-pos_y3;

		double end_speed2 = std::sqrt (xd2 * xd2 + yd2 * yd2) / TIME_STEP;
		end_accel = (end_speed - end_speed2) / TIME_STEP;
	}

	double const lastTimeHorizon = (STEP_HORIZON - previous_path_x.size ()) * TIME_STEP;


	std::vector<Candidate> candidates;
	unsigned best = 0;
	double bestScore = std::numeric_limits<double>::max ();

	/*std::cout << pos_x << "\t"
	          << pos_y << "\t"
	          << angle << std::endl;
	std::cout << end_speed * cos(angle) << "\t"
	          << end_speed * sin(angle) << "\t"
	          << end_accel * cos(angle) << "\t"
	          << end_accel * sin(angle) << std::endl;*/

	//do
	//{
	/*for (unsigned i = 0; i < NUM_TRAJECTORIES; ++i)
	{
		Candidate const c = generateTrajectory (
			carState, desiredManeuver, defaultManeuverDistance,
			(desiredManeuver.secondsToReachTarget_ <= 0)
			? defaultManeuverHorizon
			: desiredManeuver.secondsToReachTarget_,
			pos_x, pos_y, angle, end_speed, end_accel);

		if (! c.isValid)
			continue;

		if (c.score < bestScore)
		{
			best = candidates.size ();
			bestScore = c.score;
		}

		candidates.push_back (c);
	}
	//} while (candidates.size () == 0);

	assert (candidates.size () > 0);
	std::cout << "Num Candidates: " << candidates.size ()
	          << " best score: " << bestScore << std::endl;*/

#if 0
	std::vector<double> const &xcoeffs = candidates[best].xc;
	std::vector<double> const &ycoeffs = candidates[best].yc;

	for (int i = 1; i < STEP_HORIZON - previous_path_size; ++i)
	{
		double const x = evalPoly (xcoeffs, i * TIME_STEP);
		double const y = evalPoly (ycoeffs, i * TIME_STEP);

		t.x.push_back (x);
		t.y.push_back (y);
	}
#else
	//carState, desiredManeuver, defaultManeuverDistance,
	double const T = (desiredManeuver.secondsToReachTarget_ <= 0)
					 ? defaultManeuverHorizon
					 : desiredManeuver.secondsToReachTarget_;
	//pos_x, pos_y, angle, end_speed, end_accel

	auto const sd = getFrenet(pos_x, pos_y, angle, worldModel_.map());
	double const currentS = sd[0];
	double const currentD = sd[1];

	double const desiredS = carState.s + defaultManeuverDistance;
	double const desiredD = laneToD(desiredManeuver.targetLaneId_);

	double s = currentS;
	double d = currentD;

	std::vector<double> splineX;
	std::vector<double> splineY;

	//std::cout << "Test Points\n";

	for (unsigned i = 0; i < 6; ++i)
	{
		std::vector<double> const xy = getXY(s, d, worldModel_.map());

		double const shift_x = xy[0] - pos_x;
		double const shift_y = xy[1] - pos_y;

		splineX.push_back(shift_x * cos(0 - angle) - shift_y * sin(0 - angle));
		splineY.push_back(shift_x * sin(0 - angle) + shift_y * cos(0 - angle));

		/*std::cout << "\t" << xy[0] << "->" << splineX.back ()
		          << "\t" << xy[1] << "->" << splineY.back () << "\n";*/

		// Spacing of 20m is set here, speed is handled later
		/// \todo Tweak spacing for candidate generation?
		s += 20;
		d = ramp (d, desiredD, 1);
	}

	//std::cout << std::endl;

	tk::spline spline;

	spline.set_points (splineX, splineY);

	double constexpr ACCEL = 0.1;

	double x = 0;
	double v = end_speed;

	for (int i = 1; i < STEP_HORIZON - previous_path_size; ++i)
	{
		// x, y in vehicle space
		x = x + v * TIME_STEP;
		double const y = spline (x);

		/*std::cout << "\tspeed\t" << v << "\n"
		          << "\tx\t" << x << "\n"
		          << "\ty\t" << y << "\n"
		          << std::endl;*/

		v = ramp(v, desiredManeuver.targetSpeed_, ACCEL);

		// now convert to world space
		double const wx = (x * cos(angle) - y * sin(angle)) + pos_x;
		double const wy = (x * sin(angle) + y * cos(angle)) + pos_y;

		assert (t.x.empty () || (fabs(wx - t.x.back()) < SPEED_LIMIT));

		t.x.push_back(wx);
		t.y.push_back(wy);
	}

	/*std::cout << "Num Points: " << t.x.size() << " " << t.y.size() << std::endl;

	for (unsigned i = 0; i < t.x.size(); ++i)
	{
		std::cout << t.x[i] << "\t" << t.y[i] << "\n";
	}

	std::cout << std::endl;*/


	/*Candidate const &bc = candidates[best];
	for (int i = 1; i < STEP_HORIZON - previous_path_size; ++i)
	{
		double const x = bc.xc[i];
		double const y = bc.yc[i];

		t.x.push_back (x);
		t.y.push_back (y);
	}*/
#endif

	return t;
}













TrajectoryPlanner::Candidate TrajectoryPlanner::generateTrajectory (
	CarState const &carState, Maneuver const &desiredManeuver,
	double const distance, double const time,
	double const pos_x, double const pos_y, double const angle,
	double const end_speed, double const end_accel) const
{
	double const distanceHorizon = std::max (10.0, distance + sRand_(randEngine_));
	double const desiredD = laneToD(desiredManeuver.targetLaneId_) + dRand_(randEngine_);
	double const desiredS = carState.s + distanceHorizon;

	// Make the JMT in x-y space rather than s-d space to ensure we minimize Jerk in real world
	// space and not in an artificial space.
	std::vector<double> p1 = getXY(desiredS, desiredD, worldModel_.map());

	Candidate c;

	double const timeHorizon = std::max(0.1, time + timeRand_(randEngine_));

	// Try varying speed, accel, and d value
	c.xc = JMT ({pos_x, end_speed * cos(angle), end_accel * cos(angle)},
	            {p1[0], desiredManeuver.targetSpeed_ * cos(p1[2]), 0},
	            timeHorizon);
	c.yc = JMT ({pos_y, end_speed * sin(angle), end_accel * sin(angle)},
	            {p1[1], desiredManeuver.targetSpeed_ * sin(p1[2]), 0},
	            timeHorizon);

	Score const s = scoreCandidate (c, desiredD, desiredManeuver);

	c.score = s.score;
	c.isValid = s.isValid;

	return std::move (c);
}

TrajectoryPlanner::Score TrajectoryPlanner::scoreCandidate (
	Candidate const &c, double const desiredD, Maneuver const &desiredManeuver) const
{
	//x = c0 + tc1 + t2c2 + t3c3 + t4c4 + t5c5;
	//v = c1 + 2tc2 + 3t2c3 + 4t3c4 + 5t4c5;
	//a = 2c2 + 6tc3 + 12t2c4 + 20t3c5; <= 0, v is at a local min/max
	//j = 6c3 + 24tc4 + 60t2c5; <= 0, a is at a local min/max

	std::vector<double> vxc = {c.xc[1], 2*c.xc[2], 3*c.xc[3], 4*c.xc[4], 5*c.xc[5]};
	std::vector<double> vyc = {c.yc[1], 2*c.yc[2], 3*c.yc[3], 4*c.yc[4], 5*c.yc[5]};

	std::vector<double> axc = {2*c.xc[2], 6*c.xc[3], 12*c.xc[4], 20*c.xc[5]};
	std::vector<double> ayc = {2*c.yc[2], 6*c.yc[3], 12*c.yc[4], 20*c.yc[5]};

	std::vector<double> jxc = {6*c.xc[3], 24*c.xc[4], 60*c.xc[5]};
	std::vector<double> jyc = {6*c.yc[3], 24*c.yc[4], 60*c.yc[5]};

	// Use the square of the values to save some cycles
	double minv2 = std::numeric_limits<double>::max ();
	double maxv2 = 0;
	double mina2 = std::numeric_limits<double>::max ();
	double maxa2 = 0;
	double minj2 = std::numeric_limits<double>::max ();
	double maxj2 = 0;

	for (int i = 1; i < STEP_HORIZON /*- previous_path_size*/; ++i)
	{
		double const vx = evalPoly (vxc, i * TIME_STEP);
		double const vy = evalPoly (vxc, i * TIME_STEP);
		double const v2 = vx * vx + vy * vy;

		minv2 = std::min (minv2, v2);
		maxv2 = std::max (maxv2, v2);

		double const ax = evalPoly (axc, i * TIME_STEP);
		double const ay = evalPoly (axc, i * TIME_STEP);
		double const a2 = ax * ax + ay * ay;

		mina2 = std::min (mina2, a2);
		maxa2 = std::max (maxa2, a2);

		double const jx = evalPoly (jxc, i * TIME_STEP);
		double const jy = evalPoly (jxc, i * TIME_STEP);
		double const j2 = jx * jx + jy * jy;

		minj2 = std::min (minj2, j2);
		maxj2 = std::max (maxj2, j2);
	}

	Score s;

	s.isValid = maxv2 < SPEED_LIMIT_2 && maxa2 < ACCEL_LIMIT_2 && maxj2 < JERK_LIMIT_2;

	// Prioritize being close to the speed limit
	double constexpr GAIN_SPEED = 1.0; // bound to 22
	// Prioritize minimizing lateral jerk over longitudinal jerk
	double constexpr GAIN_MIN_JERK = 1.0; // bound to 10
	// Prioritize maximal distance to obstacles
	// Prioritize center of lane
	double constexpr GAIN_CENTER = 2.0; // bound to maybe 4?

	if (c.isValid)
		s.score =
			GAIN_SPEED * (std::sqrt(SPEED_LIMIT_2) - std::sqrt(maxv2)) +
			GAIN_MIN_JERK * (maxj2) +
			GAIN_CENTER * fabs(desiredD - laneToD(desiredManeuver.targetLaneId_));
		//s.score = timeHorizon + (std::sqrt (SPEED_LIMIT_2) - std::sqrt (maxv2));
	else
		s.score = std::numeric_limits<double>::max ();

	return s;
}
