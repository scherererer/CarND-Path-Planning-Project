#include "worldModel.h"
#include "utilities.h"


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

void WorldModel::update (Target const &t)
{
	targets_[t.id ()] = t;
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
