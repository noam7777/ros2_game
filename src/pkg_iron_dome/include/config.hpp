#pragma once

#define GRID_COLUMNS            (200)
#define GRID_ROWS               (100)
#define GRID_TO_WINDOW_RATIO    (4)
#define WORLD_WIDTH             (GRID_COLUMNS * GRID_TO_WINDOW_RATIO)
#define WORLD_HEIGHT            (GRID_ROWS * GRID_TO_WINDOW_RATIO)


#define GRAVITY                 (-10) // Pixels / sec

#define RENDER_STEP_SIZE_MS     (20)
#define PHYSICS_STEP_SIZE_SEC    (0.01f)


#define PITCHER_POSITION_X   (170)//(170)
#define PITCHER_POSITION_Y   (50)//(0)
#define PITCHER_ANGLE_DEG    (120)

#define CANNON_POSITION_X   (10)
#define CANNON_POSITION_Y   (5)
#define CANNON_ANGLE_RAD    (0.7f)

#define CANNON_HEIGHT       (5)
#define CANNON_WIDTH        (5)
#define CANNON_BARREL_INIT_POSITION_X (CANNON_POSITION_X + (CANNON_WIDTH / 2))
#define CANNON_BARREL_INIT_POSITION_Y (CANNON_POSITION_Y + (CANNON_HEIGHT))
#define CANNON_ANGLE_RAD            (0.7f)

#define MISSILE_INIT_VEL_X          (0.0f)
#define MISSILE_INIT_VEL_Y          (0.0f)
#define MISSILE_MASS                (1.0f)
#define MISSILE_MOMENT_OF_INERTIA   (1.0f)
#define MISSILE_LENGTH              (1.0f)

// logs
#define LOG_MISSILE_CONTROLLER      (false)


