#include "missile.hpp"
#include <iostream>
#include <cmath>
// black box


namespace iron_dome_game
{
Missile::Missile(uint16_t missileId, Eigen::Vector2f initialPos) : mass(MISSILE_MASS), momentOfInertia(MISSILE_MOMENT_OF_INERTIA), length(MISSILE_LENGTH), id(missileId)
{
    blackBoxIteration = 0;
    mState[0] = initialPos.x();
    mState[1] = MISSILE_INIT_VEL_X;
    mState[2] = initialPos.y();
    mState[3] = MISSILE_INIT_VEL_Y;
    mState[4] = atan2f(MISSILE_INIT_VEL_Y, MISSILE_INIT_VEL_X);
    mState[5] = 0.0f;

    width   = 3;
    height  = 3;

    std::string fileName = "missile" + std::to_string(missileId) + ".csv";

    missileBlackBoxFile.open(fileName);
    if (missileBlackBoxFile.is_open()) {
        missileBlackBoxFile << "loopIteration, time (us), navState, pos[0], pos[1] \n";
        missileBlackBoxFile.flush();
    }
}

//============================================================================//
Missile::~Missile() {
    missileBlackBoxFile.close();
}
//============================================================================//


void Missile::render(sf::RenderWindow &window)
{
    auto x = (float)WORLD_WIDTH - pos().x() * (float)GRID_TO_WINDOW_RATIO;
    auto y = (float)WORLD_HEIGHT - pos().y() * (float)GRID_TO_WINDOW_RATIO;
    sf::RectangleShape line(sf::Vector2f(20.f, 2.f));
    line.rotate(radiansToDegrees(this->mState[4]));
    line.setFillColor(sf::Color(100, 250, 50));
    line.setPosition(x, y);
    window.draw(line);
}

void Missile::calculateCmd(float dtSec)
{
    if (dtSec >= 0.0001f) {
        float kpPitch = 1.0f;
        float kdPitch = 2.0f;
        float kpVel =   0.06f;
        float kdVel =   0.01f;
        float desiredVelToTarget = 30.0f;


        Eigen::Vector2f targetPos(PITCHER_POSITION_X, PITCHER_POSITION_Y);
        
        Eigen::Vector2f vectorToTarget = targetPos - pos();
        
        float pitchToTarget = atan2(vectorToTarget[1], vectorToTarget[0]);

        float pitchToTargetWrapedToPi = wrapToPi(pitchToTarget);

        float pitchError = wrapToPi((pitchToTargetWrapedToPi) - wrapToPi(mState[4]));

        float dPitchErr_dt = wrapToPi(pitchError - prevPitchError) / dtSec;
        float desiredTorque = (pitchError * kpPitch) +( dPitchErr_dt * kdPitch);
        this->cmdSideThrust = desiredTorque;

        // calc desired acc in the direction of the missile heading:
        
        float velToTarget =  this->vel().dot(vectorToTarget.normalized());
        float velError = desiredVelToTarget - velToTarget;
        float dVelErr_dt = (velError - prevVelError) / dtSec;

        cmdThrust = (velError * kpVel) +( dVelErr_dt * kdVel);

        prevPitchError = pitchError;
        prevVelError = velError;
        if (LOG_MISSILE_CONTROLLER) {
            std::cout << "pitchToTarget = " << pitchToTarget << "\n";
            std::cout << "pitchToTargetWrapedToPi = " << pitchToTargetWrapedToPi << "\n";
            std::cout << "pitchError = " << pitchError << "\n";
            std::cout << "prevPitchError = " << prevPitchError << "\n";
            std::cout << "dPitchErr_dt = " << dPitchErr_dt << "\n";
            std::cout << "desiredTorque = " << desiredTorque << "\n";
            std::cout << "\n";
        }
    }
    else {
        std::cout << "ERROR!, dtSec in controller = " << dtSec << "\n" ;
    }
}

void Missile::writeToBlackBox(float tSec)
{
    if (missileBlackBoxFile.is_open()) {
        std::string blackboxLine;
        blackboxLine +=  std::to_string(this->blackBoxIteration);
        blackboxLine +=  ", ";
        blackBoxIteration++;
        blackboxLine +=  std::to_string((uint32_t)(tSec * 1000000.f));

        blackboxLine +=  ", ";
        blackboxLine +=  "0";

        blackboxLine +=  ", ";
        blackboxLine +=  std::to_string(this->pos().x());
        blackboxLine +=  ", ";
        blackboxLine +=  std::to_string(this->pos().y());

        blackboxLine += "\n";
            this->missileBlackBoxFile << blackboxLine;
            missileBlackBoxFile.flush();
    }
}


void Missile::step(float dtSec, float tSec) {
    // std::chrono::steady_clock::time_point currentTime =  std::chrono::steady_clock::now();

    calculateCmd(dtSec);
    mState[0] = mState[0] +  mState[1] * dtSec;
    mState[1] = mState[1] + cmdThrust / mass * cos(mState[4]) * dtSec /* + cmdSideThrust / mass *sin(mState[4]) */;
    mState[2] = mState[2] +  mState[3] * dtSec;
    mState[3] = mState[3] + cmdThrust / mass * sin(mState[4]) * dtSec /* + cmdSideThrust / mass *sin(mState[4] + (pi/2)) */;
    mState[4] = std::fmod((mState[4] + (mState[5] * dtSec)), 2.0f * M_PIf32);
    mState[5] = mState[5] + cmdSideThrust / momentOfInertia * (length / 2) * dtSec;
    // test the heading of the missile:
    Eigen::Vector2f missileUnitVec(cos(mState[4]), sin(mState[4]));

    writeToBlackBox(tSec);

    return;
}

Eigen::Vector2f Missile::pos(void) 
{
    return Eigen::Vector2f(this->mState[0], this->mState[2]);
}
Eigen::Vector2f Missile::vel(void) 
{
    return Eigen::Vector2f(this->mState[1], this->mState[3]);
}

}