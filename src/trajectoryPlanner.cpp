
#include "trajectoryPlanner.h"
#include "constants.h"
#include "utilities.h"

#include "Eigen-3.3/Eigen/Dense"

#include <iostream>


namespace
{

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
{
}

TrajectoryPlanner::Trajectory TrajectoryPlanner::update (
	std::vector<double> const &previous_path_x, std::vector<double> const &previous_path_y,
	double end_path_s, double end_path_d,
	CarState const &carState, Maneuver const &desiredManeuver)
{
	int constexpr STEP_HORIZON = 50;
	Trajectory t;
	size_t const previous_path_size = std::min (previous_path_x.size (), size_t (10));

	for (unsigned i = 0; i < previous_path_size; ++i)
	{
		t.x.push_back (previous_path_x[i]);
		t.y.push_back (previous_path_y[i]);
	}

	double pos_x = 0;
	double pos_y = 0;
	double angle = 0;
	double end_speed = 0;

	if(previous_path_size == 0)
	{
		pos_x = carState.x;
		pos_y = carState.y;
		angle = deg2rad(carState.yaw);

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

		double const yd = pos_y-pos_y2;
		double const xd = pos_x-pos_x2;

		angle = atan2(yd, xd);
		end_speed = std::sqrt (xd * xd + yd * yd) / TIME_STEP;
	}

	double const defaultManeuverDistance = 66;
	double const defaultManeuverHorizon = 3;

	double const lastTimeHorizon = (STEP_HORIZON - previous_path_x.size ()) * TIME_STEP;

	// I'd rather make the JMT in x-y space than s-d space.
	std::vector<double> p1 = getXY(carState.s + defaultManeuverDistance,
								   int (carState.d), worldModel_.map ());
	std::vector<double> xcoeffs = JMT ({pos_x, end_speed * cos(angle), 0},
	                                   {p1[0], desiredManeuver.targetSpeed_ * cos(p1[2]), 0},
	                                   defaultManeuverHorizon);
	std::vector<double> ycoeffs = JMT ({pos_y, end_speed * sin(angle), 0},
	                                   {p1[1], desiredManeuver.targetSpeed_ * sin(p1[2]), 0},
	                                   defaultManeuverHorizon);

	for (int i = 1; i < STEP_HORIZON - previous_path_size; ++i)
	{
		double const x = evalPoly (xcoeffs, i * TIME_STEP);
		double const y = evalPoly (ycoeffs, i * TIME_STEP);

		t.x.push_back (x);
		t.y.push_back (y);
	}

	return t;
}
