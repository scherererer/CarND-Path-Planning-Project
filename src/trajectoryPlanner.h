
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
		Trajectory trajectory;
		double score;
		bool isValid;
	};

	Candidate generateTrajectory (Trajectory const &seedTrajectory,
	                              double pos_x, double pos_y, double angle,
	                              double current_s, double current_d,
	                              double current_speed,
	                              double desired_s, double desired_d,
	                              double desired_speed) const;

	struct Score
	{
		double score;
		bool isValid;
	};

	Score scoreCandidate (Candidate const &c, double desiredD, Maneuver const &desiredManeuver) const;

	/// \brief World model
	WorldModel const &worldModel_;

	std::random_device randDevice_;
	std::default_random_engine mutable randEngine_;
	std::normal_distribution<double> mutable sRand_;
	std::normal_distribution<double> mutable dRand_;
	std::normal_distribution<double> mutable timeRand_;
	std::normal_distribution<double> mutable vRand_;
};
