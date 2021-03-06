
#pragma once

#include "map.h"
#include "trajectory.h"

#include <chrono>
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
		        double s, double d,
		        std::chrono::time_point<std::chrono::steady_clock> time)
			: id_ (id)
			, x_ (x)
			, y_ (y)
			, vx_ (vx)
			, vy_ (vy)
			, ax_ (0.0)
			, ay_ (0.0)
			, s_ (s)
			, d_ (d)
			, speed_ (std::sqrt (vx * vx + vy * vy))
			, time_ (time)
		{
		}

		bool isValid () const
			{ return id_ >= 0; }

		explicit operator bool () const
			{ return isValid (); }

		int lane () const;

		int id () const { return id_; }
		double x () const { return x_; }
		double y () const { return y_; }
		double vx () const { return vx_; }
		double vy () const { return vy_; }
		double ax () const { return ax_; }
		double ay () const { return ay_; }
		double s () const { return s_; }
		double d () const { return d_; }

		double speed () const { return speed_; }
		Trajectory const &trajectory () const { return trajectory_; }

	private:
		int id_;
		double x_;
		double y_;
		double vx_;
		double vy_;
		double ax_;
		double ay_;
		double s_;
		double d_;

		double speed_;   ///< Total speed
		std::chrono::time_point<std::chrono::steady_clock> time_;

		Trajectory trajectory_;

		friend class WorldModel;
	};

	~WorldModel ();
	explicit WorldModel (Map const &map);

	/// \brief Update with the latest sensor fusion data
	void update (Target const &t);

	/// \param lane Lane to look at
	/// \param s Car's 'S' position
	/// \parma farthest_s The farthest to look ahead, 0 indicates infinity
	Target nextInLane (int lane, double s, double farthest_s = 0.0) const;
	/// \param lane Lane to look at
	/// \param s Car's 'S' position
	/// \parma farthest_s The farthest to look behind, 0 indicates infinity
	Target previousInLane (int lane, double s, double farthest_s = 0.0) const;
	/// \brief Get the target with the given id
	Target target (int id) const;

	/// \brief Is the target circle going to collide with a known target?
	/// \param x World x coordinate, meters
	/// \param y World y coordinate, meters
	/// \param t Time ahead from now in seconds
	/// \param radius Radius around x,y in meters to test
	bool isCollision (double x, double y, double t, double radius = 3.0) const;

	Map const &map () const
		{ return map_; }

private:
	std::unordered_map<int, Target> targets_;
	Map const &map_;
};
