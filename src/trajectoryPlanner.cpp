
#include "trajectoryPlanner.h"
#include "constants.h"
#include "utilities.h"

#include "Eigen-3.3/Eigen/Dense"

#include <iostream>
#include <limits>


namespace
{

size_t constexpr NUM_PREVIOUS_PATH_POINTS = 10;

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
double constexpr V_STD_DEV = 3.0;

double constexpr BUFFER_DISTANCE = 50;
double constexpr MAINTAIN_DISTANCE = 20;

unsigned constexpr NUM_TRAJECTORIES = 1000;

double constexpr MIN_D = laneToD(0)-1.5;
double constexpr MAX_D = laneToD(2)+1.5;

inline double speedForTarget (WorldModel::Target const &t, CarState const &carState,
                              double targetSpeed)
{
	assert (t);
	double const distanceToTarget = fabs (t.s() - carState.s);

	if (distanceToTarget < BUFFER_DISTANCE && t.speed() < targetSpeed)
	{
		double const newSpeed =
			 t.speed() -
			 clip(0.1 * (MAINTAIN_DISTANCE - distanceToTarget), 0, t.speed ());

		if (newSpeed < targetSpeed)
			return newSpeed;
	}

	return targetSpeed;
}


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

Trajectory TrajectoryPlanner::update (
	std::vector<double> const &previous_path_x, std::vector<double> const &previous_path_y,
	double end_path_s, double end_path_d,
	CarState const &carState, Maneuver const &desiredManeuver)
{
	size_t const previous_path_size =
		std::min (previous_path_x.size (), NUM_PREVIOUS_PATH_POINTS);

	double const targetSpeed = [&]()
	{
		double targetSpeed = desiredManeuver.targetSpeed_;

		// Safety for car in current lane
		if (WorldModel::Target const t =
		    worldModel_.nextInLane (getLane (carState.d), carState.s))
			targetSpeed = speedForTarget (t, carState, targetSpeed);

		// Target car
		if (WorldModel::Target const t =
			worldModel_.target (desiredManeuver.targetLeadingVehicleId_))
			targetSpeed = speedForTarget (t, carState, targetSpeed);

		return targetSpeed;
	} ();

	std::cout << "-----------------------------------\n"
	          << "Lane: " << getLane (carState.d) << " (" << carState.d  << ") -> "
	          << desiredManeuver.targetLaneId_
	          << " (" << laneToD (desiredManeuver.targetLaneId_) << ")\n"
	          << "Speed: " << carState.speed << " -> "  << targetSpeed
	          << " [" << desiredManeuver.targetSpeed_ << "]\n"
	          << "Target Vehicle: " << desiredManeuver.targetLeadingVehicleId_ << "\n"
	          << "ETA: " << desiredManeuver.secondsToReachTarget_ << "\n"
	          << std::endl;

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

	std::vector<Candidate> candidates;
	double bestScore = std::numeric_limits<double>::max ();

	/*
	unsigned best = 0;
	//do
	//{
	for (unsigned i = 0; i < NUM_TRAJECTORIES; ++i)
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

	auto const sd = getFrenet(pos_x, pos_y, angle, worldModel_.map());
	double const current_s = sd[0];
	double const current_d = sd[1];

	double const desired_s = carState.s + defaultManeuverDistance;
	double const desired_d = laneToD(desiredManeuver.targetLaneId_);

	Candidate best =
		Candidate::generate (t, worldModel_.map(),
		                     {carState.x, carState.y}, {pos_x, pos_y}, angle,
		                     current_s, current_d,
		                     end_speed,
		                     desired_s, desired_d,
		                     Candidate::TARGET_SPEED, targetSpeed);

	best.isSafe = isCandidateSafe(best);
	best.score = scoreCandidate(best, desired_d, targetSpeed);

	if (best.isSafe)
		return best.trajectory;

	// Stay in lane for emergency situations
	double const emergency_desired_d = clip(laneToD(getLane(current_d)),
	                                        laneToD(0), laneToD(2));

	// Try an emergency stop trajectory
	best = Candidate::generate (t, worldModel_.map(),
	                            {carState.x, carState.y}, {pos_x, pos_y}, angle,
	                            current_s, current_d,
	                            end_speed,
	                            desired_s, emergency_desired_d,
	                            Candidate::TARGET_SPEED, 0.0);

	best.isSafe = isCandidateSafe(best);

	std::cerr << "**************\n"
	          << "EMERGENCY STOP\n"
	          << "EMERGENCY STOP\n"
	          << "EMERGENCY STOP\n"
	          << "**************\n";

	if (! best.isSafe)
		std::cerr << " COLLISION IMMINENT " << std::endl;

	return best.trajectory;
}

bool TrajectoryPlanner::isCandidateSafe (Candidate const &c) const
{
	return (! doesCandidateCollide(c)) && doesCandidateStayOnRoad(c);
}

bool TrajectoryPlanner::doesCandidateCollide (Candidate const &c) const
{
	double time = 0.0;
	// We don't look forward more than 3 seconds because our approximations aren't that good
	double constexpr MAX_LOOKAHEAD_TIME = 3.0;

	for (int i = 0; i < c.trajectory.x.size(); ++i)
	{
		if (time > MAX_LOOKAHEAD_TIME)
			break;

		double const x = c.trajectory.x[i];
		double const y = c.trajectory.y[i];

		if (worldModel_.isCollision(x, y, time))
			return true;

		time += TIME_STEP;
	}

	return false;
}

bool TrajectoryPlanner::doesCandidateStayOnRoad (Candidate const &c) const
{
	for (int i = 1; i < c.trajectory.x.size(); ++i)
	{
		double const x0 = c.trajectory.x[i-1];
		double const y0 = c.trajectory.y[i-1];

		double const x1 = c.trajectory.x[i];
		double const y1 = c.trajectory.y[i];

		double const angle = std::atan2(y1-y0, x1-x0);

		auto const sd = getFrenet(x1, y1, angle, worldModel_.map());
		double const d = sd[1];

		if (d < MIN_D || d > MAX_D)
			return false;
	}

	return true;
}

double TrajectoryPlanner::scoreCandidate (Candidate const &c, double const desired_d,
                                          double const desired_v) const
{
	double minv2 = std::numeric_limits<double>::max ();
	double maxv2 = 0;
	double mina2 = std::numeric_limits<double>::max ();
	double maxa2 = 0;
	double minj2 = std::numeric_limits<double>::max ();
	double maxj2 = 0;

	double d_centering = 0.0;
	double v_score = 0.0;

	double lastvx = 0;
	double lastvy = 0;
	double lastax = 0;
	double lastay = 0;

	for (unsigned i = 1; i < c.trajectory.x.size (); ++i)
	{
		double const x0 = c.trajectory.x[i-1];
		double const y0 = c.trajectory.y[i-1];

		double const x1 = c.trajectory.x[i];
		double const y1 = c.trajectory.y[i];

		double const angle = std::atan2(y1-y0, x1-x0);
		auto const sd = getFrenet(x1, y1, angle, worldModel_.map());

		d_centering += (desired_d - sd[1]) * (desired_d - sd[1]);

		double const vx = (x1 - x0) / TIME_STEP;
		double const vy = (y1 - y0) / TIME_STEP;
		double const v2 = vx * vx + vy * vy;
		double const v = std::sqrt(v2);

		v_score += (desired_v - v) * (desired_v - v);

		minv2 = std::min (minv2, v2);
		maxv2 = std::max (maxv2, v2);

		if (i == 1)
		{
			lastvx = vx;
			lastvy = vy;
			continue;
		}

		double const ax = (vx - lastvx) / TIME_STEP;
		double const ay = (vy - lastvy) / TIME_STEP;
		double const a2 = ax * ax + ay * ay;

		mina2 = std::min (mina2, a2);
		maxa2 = std::max (maxa2, a2);

		if (i == 2)
		{
			lastax = ax;
			lastay = ay;
			continue;
		}

		double const jx = (ax - lastax) / TIME_STEP;
		double const jy = (ay - lastay) / TIME_STEP;
		double const j2 = jx * jx + jy * jy;

		minj2 = std::min (minj2, j2);
		maxj2 = std::max (maxj2, j2);
	}

	// Prioritize being close to the speed limit
	double constexpr GAIN_SPEED = 1.0; // bound to 22
	// Prioritize minimizing lateral jerk over longitudinal jerk
	double constexpr GAIN_MIN_JERK = 1.0; // bound to 10
	// Prioritize maximal distance to obstacles
	// Prioritize center of lane
	double constexpr GAIN_CENTER = 1.0; // bound to maybe 4?

	double constexpr GAIN_V = 1.0;

	return GAIN_SPEED * (std::sqrt(SPEED_LIMIT_2) - std::sqrt(maxv2)) +
	       GAIN_MIN_JERK * (maxj2) +
	       GAIN_CENTER * d_centering +
	       GAIN_V * v_score;
}
