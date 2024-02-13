#include "entity.hpp"

namespace iron_dome_game
{

Eigen::Vector2f SimpleEntity::pos() 
{
    if (isStatic())
    {
        return trajectory.initialState.state.pos;
    }
    else
    {
        return trajectory.calculatePosition();
    }
}

Eigen::Vector2f SimpleEntity::vel() 
{
    if (isStatic())
    {
        return Eigen::Vector2f(0.0f, 0.0f);
    }
    else
    {
        return trajectory.calculateVelocity();
    }
}

//============================================================================//

BoundingBox Entity::boundingBox() 
{
    BoundingBox bbox;
    bbox.p1 = pos();
    bbox.p2.x() = pos().x() + width /* - 1 */;
    bbox.p2.y() = pos().y() + height /* - 1 */;

    return bbox;
}

}