#include "config.hpp"
#include "cannon.hpp"

namespace iron_dome_game
{
Cannon::Cannon() : StateSpaceEntity()
{
    // trajectory.initialState.state.pos.x() = CANNON_POSITION_X;
    // trajectory.initialState.state.pos.y() = CANNON_POSITION_Y;

    mState[0] = CANNON_BARREL_INIT_POSITION_X;
    mState[1] = 0.0f;
    mState[2] = CANNON_BARREL_INIT_POSITION_Y;
    mState[3] = 0.0f;
    mState[4] = 0.0f;
    mState[5] = 0.0f;

    width    = 5;
    height   = 5;
}

Eigen::Vector2f Cannon::pos(void) 
{
    return Eigen::Vector2f(this->mState[0], this->mState[2]);
}

Eigen::Vector2f Cannon::vel(void) 
{
    return Eigen::Vector2f(this->mState[1], this->mState[3]);
}


void Cannon::step(float dtSec, float tSec) {
    // std::chrono::steady_clock::time_point currentTime =  std::chrono::steady_clock::now();
    (void) tSec;
    
    mState[1] = cmdVel.x();
    mState[3] = cmdVel.y();
    mState[0] = mState[0] +  mState[1] * dtSec;
    mState[2] = mState[2] +  mState[3] * dtSec;


    return;
}

}