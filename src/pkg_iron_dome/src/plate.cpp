#include "plate.hpp"

namespace iron_dome_game
{
Plate::Plate(Eigen::Vector2f velocity) : SimpleEntity()
{
    trajectory.initialState.state.pos.x() = PITCHER_POSITION_X;
    trajectory.initialState.state.pos.y() = PITCHER_POSITION_Y;
    trajectory.initialState.state.velocity.x() = velocity.x();
    trajectory.initialState.state.velocity.y() = velocity.y();

    width   = 3;
    height  = 3;
}

//============================================================================//

}