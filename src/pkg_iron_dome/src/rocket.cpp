#include "rocket.hpp"
#include <iostream>

namespace iron_dome_game
{
Rocket::Rocket(State targetPlateCurrentState, Eigen::Vector2f cannonCurrentPos) : SimpleEntity()
{
    trajectory.initialState.state.pos = cannonCurrentPos;
    trajectory.initialState.state.velocity = calcReleaseVelocity(targetPlateCurrentState, cannonCurrentPos);

    width   = 3;
    height  = 3;
}

//============================================================================//


float Rocket::calcReleaseVelocityMagnitude(State p, Eigen::Vector2f cannonCurrentPos) {
    float deltaX = p.pos.x() - cannonCurrentPos.x();
    float deltaY = p.pos.y() - cannonCurrentPos.y();

    float velocityMagnitude = (deltaY * p.velocity.x() - deltaX * p.velocity.y()) 
                            / (deltaY * cosf(CANNON_ANGLE_RAD) - deltaX * sinf(CANNON_ANGLE_RAD));

    return velocityMagnitude;
}

Eigen::Vector2f Rocket::calcReleaseVelocity(State p, Eigen::Vector2f cannonCurrentPos)
{

    float velocityMagnitude = calcReleaseVelocityMagnitude(p, cannonCurrentPos);


    Eigen::Vector2f rocketInitialVel = {(velocityMagnitude * cos(CANNON_ANGLE_RAD)),
                                 (velocityMagnitude * sin(CANNON_ANGLE_RAD))};

    return rocketInitialVel;
}
}