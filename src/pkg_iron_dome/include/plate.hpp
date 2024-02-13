#pragma once

#include "entity.hpp"

namespace iron_dome_game
{
struct Plate : public SimpleEntity
{
    Plate(Eigen::Vector2f velocity);
    ~Plate() = default;

    EntityType type() override { return EntityType::PLATE; }

    bool isStatic() { return false; }
};

}