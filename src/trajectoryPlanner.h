
#pragma once

#include "carState.h"
#include "maneuver.h"
#include "worldModel.h"

#include <random>
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

	Trajectory update (std::vector<double> const &previous_path_x,
	                   std::vector<double> const &previous_path_y,
	                   double end_path_s, double end_path_d,
	                   CarState const &carState, Maneuver const &desiredManeuver);

private:
	struct Candidate
	{
		std::vector<double> xc;
		std::vector<double> yc;
		double score;
		bool isValid;
	};

	Candidate generateTrajectory (
		CarState const &carState, Maneuver const &desiredManeuver,
		double const distance, double const time,
		double const pos_x, double const pos_y, double const angle,
		double const end_speed, double const end_accel);

	/// \brief World model
	WorldModel const &worldModel_;

	std::random_device randDevice_;
	std::default_random_engine randEngine_;
	std::normal_distribution<double> sRand_;
	std::normal_distribution<double> dRand_;
	std::normal_distribution<double> timeRand_;
};
