
#include "stateMachine.h"
#include "utilities.h"

#include <cmath>
#include <iostream>


///////////////////////////////////////////////////////////////////////////
namespace
{

double constexpr MIN_GAP_AHEAD = 20;
double constexpr MIN_GAP_BEHIND = 30;
double constexpr LANE_LOOKAHEAD = 50;

}


///////////////////////////////////////////////////////////////////////////
StateMachine::~StateMachine ()
{
}

StateMachine::StateMachine (WorldModel const &worldModel)
	: worldModel_ (worldModel)
	, isInitialized_ (false)
	, currentState_ (STAY_IN_LANE)
	, car_ ()
	, targetLane_ (0)
{
}

Maneuver StateMachine::update (CarState const &car)
{
	car_ = car;

	if (! isInitialized_)
	{
		targetLane_ = getLane (car_.d);
		isInitialized_ = true;
	}

	State const previousState = currentState_;

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

	std::cout << "-----------------------------------\n"
	          << "State: " << previousState << " -> " << currentState_ << std::endl;

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
	double const wantStayInLane = 0.5;
	double const wantChangeLane =
		std::tanh (5.0 - 5.0 * (std::min(SPEED_LIMIT, car_.speed) / SPEED_LIMIT));

	std::cout << "Stay: " << wantStayInLane << "\n"
	          << "Change: " << wantChangeLane << "\n";

	if ((wantChangeLane - wantStayInLane) > 0.1)
	{
		switch (fastestLane ())
		{
		case LaneChoice::LEFT:
			std::cout << " LEFT \n";
			return BEGIN_LEFT_LANE_CHANGE;
		case LaneChoice::STAY:
			std::cout << " STUCK \n";
			// Want to change but there's no faster lane
			return STAY_IN_LANE;
		case LaneChoice::RIGHT:
			std::cout << " RIGHT \n";
			return BEGIN_RIGHT_LANE_CHANGE;
		}
	}

	std::cout << " STAY \n";
	return STAY_IN_LANE;
}

StateMachine::State StateMachine::update_beginLeftLaneChange ()
{
	assert (targetLane_ > 0);
	if (targetLane_ == 0)
		return STAY_IN_LANE;

	// Check if we should abandon this maneuver
	if (fastestLane () == LaneChoice::STAY)
		return STAY_IN_LANE;

	int const desiredLane = targetLane_ - 1;

	// Verify we have a big enough gap before initiating the maneuver
	if (WorldModel::Target const t =
	    worldModel_.nextInLane (desiredLane, car_.s, MIN_GAP_AHEAD))
		return BEGIN_LEFT_LANE_CHANGE;
	if (WorldModel::Target const t =
	    worldModel_.previousInLane (desiredLane, car_.s, MIN_GAP_BEHIND))
		return BEGIN_LEFT_LANE_CHANGE;

	targetLane_ = desiredLane;

	return LEFT_LANE_CHANGE;
}

StateMachine::State StateMachine::update_leftLaneChange ()
{
	if (getLane (car_.d) == targetLane_)
		return STAY_IN_LANE;

	return LEFT_LANE_CHANGE;
}

StateMachine::State StateMachine::update_beginRightLaneChange ()
{
	assert (targetLane_ < 2);
	if (targetLane_ == 2)
		return STAY_IN_LANE;

	// Check if we should abandon this maneuver
	if (fastestLane () == LaneChoice::STAY)
		return STAY_IN_LANE;

	int const desiredLane = targetLane_ + 1;

	// Verify we have a big enough gap before initiating the maneuver
	if (WorldModel::Target const t =
	    worldModel_.nextInLane (desiredLane, car_.s, MIN_GAP_AHEAD))
		return BEGIN_RIGHT_LANE_CHANGE;
	if (WorldModel::Target const t =
	    worldModel_.previousInLane (desiredLane, car_.s, MIN_GAP_BEHIND))
		return BEGIN_RIGHT_LANE_CHANGE;

	targetLane_ = desiredLane;

	return RIGHT_LANE_CHANGE;
}

StateMachine::State StateMachine::update_rightLaneChange ()
{
	if (getLane (car_.d) == targetLane_)
		return STAY_IN_LANE;

	return RIGHT_LANE_CHANGE;
}

Maneuver StateMachine::run_stayInLane ()
{
	WorldModel::Target const t = worldModel_.nextInLane (targetLane_, car_.s);
	Maneuver m;

    m.targetLaneId_ = targetLane_;
    m.targetLeadingVehicleId_ = t.id();
    m.targetSpeed_ = SPEED_LIMIT;
    m.secondsToReachTarget_ = -1;

	return m;
}

Maneuver StateMachine::run_beginLeftLaneChange ()
{
	WorldModel::Target const t = worldModel_.nextInLane (targetLane_ - 1, car_.s);
	Maneuver m;

    m.targetLaneId_ = targetLane_;
    m.targetLeadingVehicleId_ = t.id();
    m.targetSpeed_ = SPEED_LIMIT;
    m.secondsToReachTarget_ = -1;

	return m;
}

Maneuver StateMachine::run_leftLaneChange ()
{
	WorldModel::Target const t = worldModel_.nextInLane (targetLane_, car_.s);
	Maneuver m;

    m.targetLaneId_ = targetLane_;
    m.targetLeadingVehicleId_ = t.id();
    m.targetSpeed_ = SPEED_LIMIT;
    m.secondsToReachTarget_ = -1;

	return m;
}

Maneuver StateMachine::run_beginRightLaneChange ()
{
	WorldModel::Target const t = worldModel_.nextInLane (targetLane_ + 1, car_.s);
	Maneuver m;

    m.targetLaneId_ = targetLane_;
    m.targetLeadingVehicleId_ = t.id();
    m.targetSpeed_ = SPEED_LIMIT;
    m.secondsToReachTarget_ = -1;

	return m;
}

Maneuver StateMachine::run_rightLaneChange ()
{
	WorldModel::Target const t = worldModel_.nextInLane (targetLane_, car_.s);
	Maneuver m;

    m.targetLaneId_ = targetLane_;
    m.targetLeadingVehicleId_ = t.id();
    m.targetSpeed_ = SPEED_LIMIT;
    m.secondsToReachTarget_ = -1;

	return m;
}

StateMachine::LaneChoice StateMachine::fastestLane () const
{
	double speed = SPEED_LIMIT;
	LaneChoice choice = LaneChoice::STAY;

	WorldModel::Target const t = worldModel_.nextInLane (targetLane_, car_.s,
														 LANE_LOOKAHEAD);

	if (t.isValid())
		speed = t.speed();

	if (targetLane_ > 0)
	{
		WorldModel::Target const t = worldModel_.nextInLane (targetLane_ - 1, car_.s,
															 LANE_LOOKAHEAD);

		if (t.isValid() && t.speed() > speed)
		{
			speed = t.speed();
			choice = LaneChoice::LEFT;
		}
		else if (! t.isValid()) // Nothing blocking us
		{
			speed = SPEED_LIMIT;
			choice = LaneChoice::LEFT;
		}
	}

	if (targetLane_ < 2)
	{
		WorldModel::Target const t = worldModel_.nextInLane (targetLane_ + 1, car_.s,
															 LANE_LOOKAHEAD);

		if (t.isValid() && t.speed() > speed)
		{
			speed = t.speed();
			choice = LaneChoice::RIGHT;
		}
		else if (! t.isValid()) // Nothing blocking us
		{
			speed = SPEED_LIMIT;
			choice = LaneChoice::RIGHT;
		}
	}

	return choice;
}
