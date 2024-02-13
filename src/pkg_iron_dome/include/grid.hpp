#pragma once

#include <functional>
#include <list>
#include <vector>
#include <memory>

#include "config.hpp"
#include "trajectory.hpp"
#include <SFML/Graphics.hpp>


namespace iron_dome_game
{

struct Entity;
struct Rocket;
struct Plate;
struct Missile;
struct Cannon;

// Two points, representing a rectangle around an entity
struct BoundingBox
{
    Eigen::Vector2f p1;
    Eigen::Vector2f p2;
};

struct Entities {
    std::list <std::shared_ptr<Entity>> generalEntities;
    std::shared_ptr<Cannon> cannon; 
    std::vector <std::shared_ptr<Rocket>> rockets;
    std::vector <std::shared_ptr<Plate>> plates;
    std::vector <std::shared_ptr<Missile>> missiles;
};

class Grid
{
public:
    Grid() = default;
    ~Grid() = default;

    uint16_t rows() { return GRID_ROWS; }
    uint16_t columns() { return GRID_COLUMNS; }


    void addGeneralEntity(std::shared_ptr<Entity> entity);
    void spawnCannon(std::shared_ptr<Cannon> cannon);
    void addRocket(std::shared_ptr<Rocket> rocket);
    void addPlate(std::shared_ptr<Plate> plate);
    void addMissile(std::shared_ptr<Missile> missile);

    uint16_t m_hits = 0;


    void destroyDeadEntities();

    std::vector<std::shared_ptr<Plate>> getPlates() {return m_entities.plates;};

    std::vector<std::shared_ptr<Rocket>> getRockets() {return m_entities.rockets;};

    std::vector<std::shared_ptr<Missile>> getMissiles() {return m_entities.missiles;};

    std::shared_ptr<Cannon> getCannon() {return m_entities.cannon;};


    void stepPhysics(float dt, float t);

    void render(sf::RenderWindow &window);

    void saveEntitiesFrameForRendering(void) { m_entitiesFrameForRendering = m_entities;} ;
private:
    void forEveryPixel(std::function<void(int row, int col)> function, const int rowCount = GRID_ROWS, const int columnCount = GRID_COLUMNS);

    static bool intersects(std::shared_ptr<Entity> first, std::shared_ptr<Entity> second);


    char m_grid[GRID_ROWS][GRID_COLUMNS];

    Entities m_entities;
    Entities m_entitiesFrameForRendering;
};
}