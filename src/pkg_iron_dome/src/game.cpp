#include <iostream>
#include <random>
#include <memory>
#include <math.h>

#include "game.hpp"

//graphics
#include <SFML/Graphics.hpp>

#define DEG_TO_RAD(x)   (x * 0.0174533)

namespace iron_dome_game
{

    std::chrono::steady_clock::time_point t0;
    std::chrono::duration<int64_t, std::nano> t;
    float tSec;

    bool timeInitialized = false;
    
Game::Game() 
{
    grid.addGeneralEntity(std::make_shared<iron_dome_game::Pitcher>());
    grid.spawnCannon(std::make_shared<iron_dome_game::Cannon>());

    window = new sf::RenderWindow(sf::VideoMode(WORLD_WIDTH, WORLD_HEIGHT), "My window");
    window->setKeyRepeatEnabled(false);

}

//============================================================================//

Game::~Game() {
    delete window;
}

//============================================================================//

void Game::keyboardListener() 
{

sf::Event event;


    while (gameIsActive)
    {
        // while there are pending events...
        while (window->pollEvent(event))
        {
            // check the type of the event...
            switch (event.type)
            {
                // window closed
                case sf::Event::Closed:
                    std::cout << "game window closed, terminating game\n";
                    gameIsActive = false;
                    break;

                // key pressed
                case sf::Event::KeyPressed:


                    if (event.key.code == sf::Keyboard::Key::Left)
                    {
                        std::cout << "the left key was pressed" << std::endl;
                        grid.getCannon()->cmdVel.x() = 10.0f;
                    }            
                    else if (event.key.code == sf::Keyboard::Key::Right)
                    {
                        std::cout << "the right key was pressed" << std::endl;
                        grid.getCannon()->cmdVel.x() = -10.0f;
                    }


                    if (event.key.code == sf::Keyboard::Key::Enter) {
                        isShotFired = true;
                    }
                    break;

                case sf::Event::KeyReleased:
                    if (event.key.code == sf::Keyboard::Key::Left)
                    {
                        std::cout << "the left key was relesed" << std::endl;
                        grid.getCannon()->cmdVel.x() = 0;
                    }            
                    if (event.key.code == sf::Keyboard::Key::Right)
                    {
                        std::cout << "the right key was released" << std::endl;
                        grid.getCannon()->cmdVel.x() = 0;
                    }
                    break;

                case sf::Event::MouseButtonPressed:
                    if(event.mouseButton.button == sf::Mouse::Button::Left) {
                        isShotFired = true;
                    }
                    break;

                // we don't process other types of events
                default:
                    break;
            }
        }
    }
}

//============================================================================//
void Game::physicsEngine() 
{
    t0 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastTimePhysicsTimeStamp = std::chrono::steady_clock::now();
    timeInitialized = true;


    while (gameIsActive)
    {
        t = std::chrono::steady_clock::now() - t0;
        auto dt = std::chrono::steady_clock::now() - lastTimePhysicsTimeStamp;
        tSec = std::chrono::duration_cast<std::chrono::duration<float>>(t).count();
        float dtSec = std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
        if (dtSec >= PHYSICS_STEP_SIZE_SEC) {
            grid.stepPhysics(dtSec, tSec);

            lastTimePhysicsTimeStamp = std::chrono::steady_clock::now();
        }
        // logs
        if (requestingAFrameFromPhysics) {
            if (newFrameIsReadyForRendering){
            }
            else {
                grid.saveEntitiesFrameForRendering();
                newFrameIsReadyForRendering = true;
                requestingAFrameFromPhysics = false;
            }
        }
    }
}

//============================================================================//

void Game::renderFrame(sf::RenderWindow &window)
{
    if (window.isOpen()) {

        window.clear(sf::Color::Red);
        grid.render(window);

        // end the current frame
        window.display();
    }

}

void Game::renderManager() 
{
    std::chrono::steady_clock::time_point lastTimeRefreshed = std::chrono::steady_clock::now();

    while (gameIsActive)
    {

        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastTimeRefreshed).count() >= RENDER_STEP_SIZE_MS)
        {

            if (newFrameIsReadyForRendering) {
                newFrameIsReadyForRendering = false;
            }

            if (requestingAFrameFromPhysics == false) {
                requestingAFrameFromPhysics = true;
                while (!newFrameIsReadyForRendering) {};
                renderFrame(*window);
                newFrameIsReadyForRendering = false;
            }

            lastTimeRefreshed = std::chrono::steady_clock::now();
        }
    }
}
void Game::joinAllThreads(void) {
    std::cout << "joining all threads\n";
    physicsThread.join();
    renderThread.join();
    keyboardThread.join();
}
//============================================================================//
void Game::signalHandler(int signal) {
    std::cout << "Ctrl+C detected. Cleaning up..." << std::endl;
    
    // Set the flag to signal threads to stop
    gameIsActive = false;

    // Wait for the threads to join
    joinAllThreads();
    
    // Exit the program
    exit(signal);
}
//============================================================================//

void Game::play() 
{
    gameIsActive = true;

    this->keyboardThread = std::thread(&Game::keyboardListener, this);
    this->physicsThread = std::thread(&Game::physicsEngine, this);
    this->renderThread = std::thread(&Game::renderManager, this);
    
    while (!timeInitialized){};

    while (gameIsActive)
    {
        
        if (isShotFired)
        {
            static uint16_t missileCounter = 0;
            std::shared_ptr<Plate> targetPlatePtr;

            // TODO : fix this and allocate rockets wisely
            for (const auto platePtr : this->grid.getPlates()) {
                if (Rocket::calcReleaseVelocityMagnitude(platePtr.get()->trajectory.calculateCurrentState(), grid.getCannon()->pos()) > 0)
                targetPlatePtr = platePtr;
                break;
            }

            if (targetPlatePtr) {
                spawnRocket(targetPlatePtr.get()->trajectory.calculateCurrentState());
                ++shotsFired;
                
            }
            grid.addMissile(std::make_shared<Missile>(missileCounter, grid.getCannon()->pos()));
            missileCounter++;
            isShotFired = false;
        }

        grid.destroyDeadEntities();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (tSec > GAME_RUN_TIME_SEC)
        {
            gameIsActive = false;
        }
        else if (tSec / 2 > platesFired)
        {
            spawnPlate();
            ++platesFired;
        }
    }

    joinAllThreads();
    std::cout << "Game over. Total hits: " << grid.m_hits << ". Total shots fired: " << shotsFired << std::endl;
    std::cout << "Accuracy " << (float)grid.m_hits /  (float)shotsFired * 100.f << "%" << std::endl;
}

//============================================================================//

void Game::spawnPlate() 
{
    int firePower = std::rand() % 15 + 30;
    Eigen::Vector2f velocity;
    velocity.x() = std::cos(DEG_TO_RAD(PITCHER_ANGLE_DEG)) * firePower;
    velocity.y() = std::sin(DEG_TO_RAD(PITCHER_ANGLE_DEG)) * firePower;
    grid.addPlate(std::make_shared<Plate>(velocity));
}

void Game::spawnRocket(State plateCurrentState) 
{
    grid.addRocket(std::make_shared<Rocket>(plateCurrentState, grid.getCannon()->pos()));

}
}