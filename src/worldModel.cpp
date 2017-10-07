#include "worldModel.h"
#include "utilities.h"


///////////////////////////////////////////////////////////////////////////
int WorldModel::Target::lane () const
{
	return getLane (d);
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
	targets_[t.id] = t;
}

WorldModel::Target WorldModel::nextInLane (int lane, double s) const
{
	Target closest;

	for (auto const &pair : targets_)
	{
		Target const &t = pair.second;

		if (t.lane () != lane)
			continue;

		if (closest.isValid () && t.s >= s && t.s < closest.s)
			closest = t;
	}

	return closest;
}
