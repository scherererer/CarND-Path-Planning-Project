
#pragma once

#include "map.h"

#include <cmath>
#include <unordered_map>

class WorldModel
{
public:
	class Target
	{
	public:
		Target ()
			: id_ (-1) // invalid target
			, x_ (0.0)
			, y_ (0.0)
			, vx_ (0.0)
			, vy_ (0.0)
			, s_ (0.0)
			, d_ (0.0)
			, speed_ (0.0)
		{
		}

		Target (int id, double x, double y, double vx, double vy,
		        double s, double d)
			: id_ (id)
			, x_ (x)
			, y_ (y)
			, vx_ (vx)
			, vy_ (vy)
			, s_ (s)
			, d_ (d)
			, speed_ (std::sqrt (vx * vx + vy * vy))
		{
		}

		bool isValid () const
			{ return id_ >= 0; }

		int lane () const;

		int id () const { return id_; }
		double x () const { return x_; }
		double y () const { return y_; }
		double vx () const { return vx_; }
		double vy () const { return vy_; }
		double s () const { return s_; }
		double d () const { return d_; }

		double speed () const { return speed_; }

	private:
		int id_;
		double x_;
		double y_;
		double vx_;
		double vy_;
		double s_;
		double d_;

		double speed_;   ///< Total speed
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
