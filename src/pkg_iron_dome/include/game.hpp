#pragma once

#include <iostream>
#include <thread>

#include "grid.hpp"
#include "pitcher.hpp"
#include "plate.hpp"
#include "cannon.hpp"
#include "rocket.hpp"
#include "missile.hpp"

namespace iron_dome_game
{
struct Game
{
    Game();
    ~Game();
    

    void play();
    bool requestingAFrameFromPhysics = false;
    bool newFrameIsReadyForRendering = false;
    void renderFrame(sf::RenderWindow &window);
    void keyboardListener();
    void physicsEngine();
    void renderManager();
    void joinAllThreads(void);
    void signalHandler(int signal);
    Grid grid;
    sf::RenderWindow* window;


    void spawnPlate();
    void spawnRocket(State plateCurrentState);

    bool isShotFired = false;
    bool gameIsActive = false;
    

    static constexpr const int GAME_RUN_TIME_SEC = 25.0f;

    // Statistics
    uint16_t platesFired = 0;
    uint16_t shotsFired  = 0;

private:

    std::thread physicsThread;
    std::thread renderThread;
    std::thread keyboardThread;
};

}