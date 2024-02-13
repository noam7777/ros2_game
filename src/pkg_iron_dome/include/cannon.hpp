#pragma once

#include "entity.hpp"

namespace iron_dome_game
{
struct Cannon : public StateSpaceEntity
{
    Cannon();
    ~Cannon() = default;

    Eigen::Vector2f cmdVel;

    EntityType type() override { return EntityType::CANNON; }


    Eigen::Vector2f pos() override;
    Eigen::Vector2f vel() override;

    // TODO: make private:
    void step(float dt, float t) override;
    

};

}