#include "worldModel.h"

WorldModel::~WorldModel ()
{
}

WorldModel::WorldModel ()
{
}

void WorldModel::update (Target const &t)
{
	targets_[t.id] = t;
}

WorldModel::Target WorldModel::nextInLane (unsigned lane)
{
}
