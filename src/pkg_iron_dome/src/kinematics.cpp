#include "kinematics.hpp"
namespace iron_dome_game
{

float degreesToRadians(float degrees) {
    return degrees * M_PI / 180.0f;
}

float radiansToDegrees(float radians) {
    return ((radians / M_PI) * 180.0f);
}

float wrapToPi(float angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }

    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }

    return angle;
}

}