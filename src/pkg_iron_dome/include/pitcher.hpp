#pragma once

#include "entity.hpp"

namespace iron_dome_game
{
struct Pitcher : public SimpleEntity
{
    Pitcher();
    ~Pitcher() = default;

    EntityType type() override { return EntityType::PITCHER; }

    bool isStatic() { return true; }
};

}