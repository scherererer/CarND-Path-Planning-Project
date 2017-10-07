
#pragma once

#include "map.h"

#include <unordered_map>

class WorldModel
{
public:
	struct Target
	{
		Target ()
			: id (-1) // invalid target
			, x (0.0)
			, y (0.0)
			, vx (0.0)
			, vy (0.0)
			, s (0.0)
			, d (0.0)
		{
		}

		int id;
		double x;
		double y;
		double vx;
		double vy;
		double s;
		double d;

		bool isValid () const
			{ return id >= 0; }

		int lane () const;
	};

	~WorldModel ();
	explicit WorldModel (Map const &map);

	/// \brief Update with the latest sensor fusion data
	void update (Target const &t);

	Target nextInLane (int lane, double s) const;

	Map const &map () const
		{ return map_; }

private:
	std::unordered_map<int, Target> targets_;
	Map const &map_;
};
