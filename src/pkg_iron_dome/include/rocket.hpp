#pragma once

#include "entity.hpp"

namespace iron_dome_game
{
struct Rocket : public SimpleEntity
{
    Rocket(State targetPlateCurrentState, Eigen::Vector2f cannonCurrentPos);
    ~Rocket() = default;

    EntityType type() override { return EntityType::ROCKET; }

    bool isStatic() { return false; }

    static float calcReleaseVelocityMagnitude(State targetPlateCurrentState, Eigen::Vector2f cannonCurrentPos);

    Eigen::Vector2f calcReleaseVelocity(State targetPlateCurrentState, Eigen::Vector2f cannonCurrentPos);
};

}