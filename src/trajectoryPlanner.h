
#pragma once

#include "carState.h"
#include "maneuver.h"
#include "trajectory.h"
#include "worldModel.h"

#include <random>
#include <vector>

class TrajectoryPlanner
{
public:
	~TrajectoryPlanner ();
	explicit TrajectoryPlanner (WorldModel const &worldModel);

	Trajectory update (std::vector<double> const &previous_path_x,
	                   std::vector<double> const &previous_path_y,
	                   double end_path_s, double end_path_d,
	                   CarState const &carState, Maneuver const &desiredManeuver);

private:
	bool isCandidateSafe (Candidate const &c) const;
	bool doesCandidateCollide (Candidate const &c) const;
	bool doesCandidateStayOnRoad (Candidate const &c) const;

	struct Score
	{
		double score;
		bool isValid;
	};

	double scoreCandidate (Candidate const &c, double desired_d, double desired_v) const;
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
