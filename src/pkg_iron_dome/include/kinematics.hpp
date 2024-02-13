#pragma once
#include <stdint.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace iron_dome_game
{

struct State
{
    Eigen::Vector2f pos;
    Eigen::Vector2f velocity;
};

float degreesToRadians(float degrees);
float radiansToDegrees(float radians);
float wrapToPi(float angle);

}