#include "worldModel.h"
#include "utilities.h"

#include <iostream>


///////////////////////////////////////////////////////////////////////////
int WorldModel::Target::lane () const
{
	return getLane (d_);
}


///////////////////////////////////////////////////////////////////////////
WorldModel::~WorldModel ()
{
}

WorldModel::WorldModel (Map const &map)
	: map_ (map)
{
}

void WorldModel::update (Target const &update)
{
	Target &target = targets_[update.id ()];

	if (! target)
	{
		target = update;
		return;
	}

	double const dt = std::chrono::duration<double> (update.time_ - target.time_).count ();

	double const ax = (update.vx_ - target.vx_) / dt;
	double const ay = (update.vy_ - target.vy_) / dt;

	//double const angle = std::atan2(update.y_ - target.y_, update.x_ - target.x_);
	double const angle = std::atan2(update.vy_, update.vx_);
	double const a_factor = fabs(angleDiff(angle, std::atan2(ay, ax))) > M_PI/2.0;
	double const a = a_factor * std::sqrt(ax * ax + ay * ay);
	double const v = std::sqrt(update.vx_ * update.vx_ + update.vy_ * update.vy_);

	target = update;

	target.ax_ = ax;
	target.ay_ = ay;

	if (target.d_ > 0)
		target.trajectory_ =
			Candidate::generate (Trajectory(), map_,
								 { target.x_, target.y_ },
								 { target.x_, target.y_ }, angle,
								 target.s_, target.d_, v,
								 fmod(target.s_ + 60, map_.waypoints_s.back()), target.d_,
								 Candidate::TARGET_ACCEL, a).trajectory;
}

WorldModel::Target WorldModel::nextInLane (int lane, double s, double farthest_s) const
{
	Target closest;

	for (auto const &pair : targets_)
	{
		Target const &t = pair.second;

		if (t.lane () != lane)
			continue;

		if (t.s() >= s && (t.s() < s + farthest_s || farthest_s <= 0)
		    && (! closest.isValid() || t.s() < closest.s()))
			closest = t;
	}

	return closest;
}

WorldModel::Target WorldModel::previousInLane (int lane, double s, double farthest_s) const
{
	Target closest;

	for (auto const &pair : targets_)
	{
		Target const &t = pair.second;

		if (t.lane () != lane)
			continue;

		if (t.s() <= s && (t.s() > s - farthest_s || farthest_s <= 0)
		    && (! closest.isValid() || t.s() > closest.s()))
			closest = t;
	}

	return closest;
}

WorldModel::Target WorldModel::target (int id) const
{
	auto const iter = targets_.find (id);

	if (iter != targets_.end ())
		return iter->second;
	return {};
}

bool WorldModel::isCollision (double x, double y, double t, double radius) const
{
	double const radius2 = radius * radius;
	size_t const index = t / TIME_STEP;

	for (auto const &pair : targets_)
	{
		Target const &target = pair.second;

		if (target.trajectory_.x.empty ())
			continue;

		assert (index < target.trajectory_.x.size ());
		if (index >= target.trajectory_.x.size ())
			continue;

		double const dx = target.trajectory_.x[index] - x;
		double const dy = target.trajectory_.y[index] - y;

		// Simple linear model for target's position at t
		//double const tx = target.x() + target.vx()*t + 0.5*target.ax()*t*t;
		//double const ty = target.y() + target.vy()*t + 0.5*target.ay()*t*t;

		//double const dx = tx - x;
		//double const dy = ty - y;

		if (dx * dx + dy * dy < radius2)
			return true;
	}

	return false;
}
