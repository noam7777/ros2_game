#pragma once
#include <stdint.h>
#include <chrono>
#include <math.h>
#include "config.hpp"
#include "kinematics.hpp"

namespace iron_dome_game
{
struct InitialState
{
    State state;
    std::chrono::steady_clock::time_point t0;
};


struct Trajectory
{
    InitialState initialState;

    std::chrono::duration<float> duration() { return  std::chrono::steady_clock::now() - initialState.t0; }

    Eigen::Vector2f calculatePosition(std::chrono::steady_clock::time_point = std::chrono::steady_clock::now())
    {
        Eigen::Vector2f pos;
        pos.x() = (initialState.state.pos.x() + initialState.state.velocity.x() * duration().count());
        pos.y() = (initialState.state.pos.y() + initialState.state.velocity.y() * duration().count() + 0.5 * GRAVITY * pow(duration().count(), 2));

        return pos;
    }

    Eigen::Vector2f calculateVelocity(std::chrono::steady_clock::time_point = std::chrono::steady_clock::now()) {
        Eigen::Vector2f velocity;
        velocity.x() = round(initialState.state.velocity.x());
        velocity.y() = round(initialState.state.velocity.y() + GRAVITY *  duration().count());

        return velocity;
    }

    State calculateCurrentState(std::chrono::steady_clock::time_point = std::chrono::steady_clock::now()) {
        State currentState;

        currentState.pos = calculatePosition();
        currentState.velocity = calculateVelocity();
        return currentState;
    }
};
}