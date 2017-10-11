
#include "stateMachine.h"
#include "utilities.h"


///////////////////////////////////////////////////////////////////////////
namespace
{

double constexpr BUFFER_DISTANCE = 20;

}


///////////////////////////////////////////////////////////////////////////
StateMachine::~StateMachine ()
{
}

StateMachine::StateMachine (WorldModel const &worldModel)
	: worldModel_ (worldModel)
	, currentState_ (STAY_IN_LANE)
	, car_ ()
{
}

Maneuver StateMachine::update (CarState const &car)
{
	car_ = car;

	switch (currentState_)
	{
	case STAY_IN_LANE:
		currentState_ = update_stayInLane ();
		break;
	case BEGIN_LEFT_LANE_CHANGE:
		currentState_ = update_beginLeftLaneChange ();
		break;
	case LEFT_LANE_CHANGE:
		currentState_ = update_leftLaneChange ();
		break;
	case BEGIN_RIGHT_LANE_CHANGE:
		currentState_ = update_beginRightLaneChange ();
		break;
	case RIGHT_LANE_CHANGE:
		currentState_ = update_rightLaneChange ();
		break;
	}

	switch (currentState_)
	{
	case STAY_IN_LANE:
		return run_stayInLane ();
	case BEGIN_LEFT_LANE_CHANGE:
		return run_beginLeftLaneChange ();
	case LEFT_LANE_CHANGE:
		return run_leftLaneChange ();
	case BEGIN_RIGHT_LANE_CHANGE:
		return run_beginRightLaneChange ();
	case RIGHT_LANE_CHANGE:
		return run_rightLaneChange ();
	}
}

StateMachine::State StateMachine::update_stayInLane ()
{
	return STAY_IN_LANE;
}

StateMachine::State StateMachine::update_beginLeftLaneChange ()
{
	return BEGIN_LEFT_LANE_CHANGE;
}

StateMachine::State StateMachine::update_leftLaneChange ()
{
	return LEFT_LANE_CHANGE;
}

StateMachine::State StateMachine::update_beginRightLaneChange ()
{
	return BEGIN_RIGHT_LANE_CHANGE;
}

StateMachine::State StateMachine::update_rightLaneChange ()
{
	return RIGHT_LANE_CHANGE;
}

Maneuver StateMachine::run_stayInLane ()
{
	int const currentLane = getLane (car_.d);
	WorldModel::Target const t = worldModel_.nextInLane (currentLane, car_.s);
	Maneuver m;

    m.targetLaneId_ = currentLane;
    m.targetLeadingVehicleId_ = t.id();
    m.targetSpeed_ = SPEED_LIMIT;
    m.secondsToReachTarget_ = -1;

	if (fabs (t.s() - car_.s) < BUFFER_DISTANCE && t.speed() < m.targetSpeed_)
		m.targetSpeed_ = t.speed();

	return m;
}

Maneuver StateMachine::run_beginLeftLaneChange ()
{
	Maneuver m;
	return m;
}

Maneuver StateMachine::run_leftLaneChange ()
{
	Maneuver m;
	return m;
}

Maneuver StateMachine::run_beginRightLaneChange ()
{
	Maneuver m;
	return m;
}

Maneuver StateMachine::run_rightLaneChange ()
{
	Maneuver m;
	return m;
}

