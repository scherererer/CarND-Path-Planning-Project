
#pragma once

#include "carState.h"
#include "maneuver.h"
#include "worldModel.h"

class StateMachine
{
public:
	enum State
	{
		STAY_IN_LANE,
		BEGIN_LEFT_LANE_CHANGE,
		LEFT_LANE_CHANGE,
		BEGIN_RIGHT_LANE_CHANGE,
		RIGHT_LANE_CHANGE,
	};

	~StateMachine ();
	explicit StateMachine (WorldModel const &worldModel);

	Maneuver update (CarState const &car);

private:
	State update_stayInLane ();
	State update_beginLeftLaneChange ();
	State update_leftLaneChange ();
	State update_beginRightLaneChange ();
	State update_rightLaneChange ();

	Maneuver run_stayInLane ();
	Maneuver run_beginLeftLaneChange ();
	Maneuver run_leftLaneChange ();
	Maneuver run_beginRightLaneChange ();
	Maneuver run_rightLaneChange ();

	/// \brief World model
	WorldModel const &worldModel_;

	/// \brief Is this state machine initialized?
	bool isInitialized_;

	/// \brief Behavioral state of the car
	State currentState_;
	/// \brief Pose of the car
	CarState car_;

	/// \brief Current lane target
	int targetLane_;
};

