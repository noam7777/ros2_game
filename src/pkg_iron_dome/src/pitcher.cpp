#include "config.hpp"
#include "pitcher.hpp"

namespace iron_dome_game
{
Pitcher::Pitcher() : SimpleEntity()
{
    trajectory.initialState.state.pos.x() = PITCHER_POSITION_X;
    trajectory.initialState.state.pos.y() = PITCHER_POSITION_Y;

    width    = 6;
    height   = 5;
}

//============================================================================//

}