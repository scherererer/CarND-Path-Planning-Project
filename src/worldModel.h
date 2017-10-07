
#pragma once

#include <unordered_map>

class WorldModel
{
public:
	struct Target
	{
		int id;
		double x;
		double y;
		double vx;
		double vy;
		double s;
		double d;
	};

	~WorldModel ();
	WorldModel ();

	/// \brief Update with the latest sensor fusion data
	void update (Target const &t);

	Target nextInLane (unsigned lane);

private:
	std::unordered_map<int, Target> targets_;
};
