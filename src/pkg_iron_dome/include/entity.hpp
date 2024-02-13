#pragma once

#include "config.hpp"
#include "grid.hpp"
#include "trajectory.hpp"
#include "kinematics.hpp"

namespace iron_dome_game
{
enum EntityType
{
    NONE,
    PITCHER,
    CANNON,
    PLATE,
    ROCKET,
    MISSILE
};

struct Entity
{
    virtual EntityType type() { return EntityType::NONE; }

    virtual Eigen::Vector2f pos() = 0;
    virtual Eigen::Vector2f vel() = 0;

    uint16_t width = 0;
    uint16_t height = 0;

    BoundingBox boundingBox();

    bool isDead = false;

    virtual void render(sf::RenderWindow &window) {
        auto x = (float)WORLD_WIDTH - pos().x() * (float)GRID_TO_WINDOW_RATIO;
        auto y = (float)WORLD_HEIGHT - pos().y() * (float)GRID_TO_WINDOW_RATIO;
        sf::CircleShape shape(5.f);
        shape.setFillColor(sf::Color(100, 250, 50));
        shape.setOutlineThickness(1.0f);
        shape.setOutlineColor(sf::Color(250, 150, 100));
        shape.setPosition(x, y);
        window.draw(shape);
    }
};

struct SimpleEntity : public Entity {
    Trajectory trajectory;

    Eigen::Vector2f pos() override;
    Eigen::Vector2f vel() override;
    
    virtual bool isStatic() = 0;

    SimpleEntity(){
       this->trajectory.initialState.t0 = std::chrono::steady_clock::now();
    }
};

struct StateSpaceEntity : public Entity {
    Eigen::VectorXd mState = Eigen::VectorXd(6);

    std::chrono::steady_clock::time_point previousUpdateTimeStamp;

    virtual void step(float dt, float t) = 0;

    std::chrono::duration<float> durationSinceLastTimeStamp(std::chrono::steady_clock::time_point currentTimeStamp);

    StateSpaceEntity(){
        this->previousUpdateTimeStamp = std::chrono::steady_clock::now();
    };
};

}