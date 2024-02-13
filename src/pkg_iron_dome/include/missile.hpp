#pragma once

#include "entity.hpp"
#include <fstream>
#include <experimental/filesystem>

namespace iron_dome_game
{
struct Missile : public StateSpaceEntity
{
    float mass;
    float momentOfInertia;
    float length;
    uint16_t id;
    uint32_t blackBoxIteration;

    Missile(uint16_t missileId, Eigen::Vector2f initialPos);
    ~Missile();

    float cmdThrust;
    float cmdSideThrust;

    EntityType type() override { return EntityType::MISSILE; }
    virtual void render(sf::RenderWindow &window) override;

    void step(float dt, float t) override;
    void calculateCmd(float dt);

    Eigen::Vector2f pos() override;
    Eigen::Vector2f vel() override;

    // controller - todo, create a controller class
    float prevPitchError = 0;
    float prevVelError = 0;

    void writeToBlackBox(float t);
    std::ofstream missileBlackBoxFile;
};

}