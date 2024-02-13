#include <iostream>
#include "grid.hpp"
#include "entity.hpp"
#include "plate.hpp"
#include "rocket.hpp"
#include "missile.hpp"
#include "cannon.hpp"

namespace iron_dome_game
{

void Grid::addGeneralEntity(std::shared_ptr<Entity> entity) {

    m_entities.generalEntities.push_back(entity);
}

//============================================================================//

void Grid::spawnCannon(std::shared_ptr<Cannon> cannon) {

    m_entities.cannon = cannon;
}

//============================================================================//

void Grid::addRocket(std::shared_ptr<Rocket> rocket) {
    m_entities.rockets.push_back(rocket);
}

//============================================================================//

void Grid::addPlate(std::shared_ptr<Plate> plate) {
    m_entities.plates.push_back(plate);
}

//============================================================================//

void Grid::addMissile(std::shared_ptr<Missile> missile) {
    m_entities.missiles.push_back(missile);
}

//============================================================================//

void Grid::forEveryPixel(std::function<void(int row, int col)> function, const int rowCount, const int columnCount) 
{
    for (int i = 0; i < rowCount; ++i)
    {
        for (int j = 0; j < columnCount; ++j)
        {
            function(i, j);
        }
    }
}

//============================================================================//

void Grid::destroyDeadEntities() 
{
    for (auto platePtr = this->m_entities.plates.begin(); platePtr != this->m_entities.plates.end();platePtr++) {
        for (auto rocketPtr = this->m_entities.rockets.begin(); rocketPtr != this->m_entities.rockets.end(); rocketPtr++) {
            // float debug_calculatedDistance = platePtr->pos()
            if (intersects(*platePtr, *rocketPtr)) {
                if (!platePtr->get()->isDead) {
                    platePtr->get()->isDead = true;
                    this->m_hits++;
                }
                rocketPtr->get()->isDead = true;
                std::cout << "hit"<< std::endl;
            }
        }
    }

    for (auto it = this->m_entities.plates.begin(); it != this->m_entities.plates.end();) {
        if ((it->get()->trajectory.calculatePosition().y() < 0.0f) || (it->get()->isDead)) {
            it = m_entities.plates.erase(it);
        }
        else {
            ++it;
        }
    }


    for (auto it = this->m_entities.rockets.begin(); it != this->m_entities.rockets.end();) {
        if ((it->get()->trajectory.calculatePosition().y() < 0) || (it->get()->isDead)) {
            it = m_entities.rockets.erase(it);
        }
        else {
            ++it;
        }
    }

}

//============================================================================//

bool Grid::intersects(std::shared_ptr<Entity> first, std::shared_ptr<Entity> second)
{
    BoundingBox firstBoundingBox = first->boundingBox();
    // Check for overlap along the x-axis
    bool xOverlap = (first->boundingBox().p1.x() < second->boundingBox().p2.x()) && (first->boundingBox().p2.x() > second->boundingBox().p1.x());

    // Check for overlap along the y-axis
    bool yOverlap = (first->boundingBox().p1.y() < second->boundingBox().p2.y()) && (first->boundingBox().p2.y() > second->boundingBox().p1.y());

    return (xOverlap && yOverlap);
}

void Grid::stepPhysics(float dtSec, float tSec)
{
    for (auto missile : m_entities.missiles)
    {
        missile->step(dtSec, tSec);
    }
    m_entities.cannon->step(dtSec, tSec);
}

void Grid::render(sf::RenderWindow &window) {


    for (auto entity : m_entitiesFrameForRendering.generalEntities)
    {
        entity->render(window);
    }

    for (auto plate : m_entities.plates)
    {
        plate->render(window);
    }

    for (auto rocket : m_entities.rockets)
    {
        rocket->render(window);
    }

    for (auto missile : m_entities.missiles)
    {
        missile->render(window);
    }

    m_entities.cannon->render(window);
}

}