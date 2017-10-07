
#pragma once

struct Maneuver
{
    unsigned targetLaneId_;
    int targetLeadingVehicleId_;
    double targetSpeed_;
    double secondsToReachTarget_;
};
