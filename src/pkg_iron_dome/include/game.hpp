#pragma once

#include <iostream>
#include <thread>

#include "grid.hpp"
#include "pitcher.hpp"
#include "plate.hpp"
#include "cannon.hpp"
#include "rocket.hpp"
#include "missile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
namespace iron_dome_game
{
struct Game : public rclcpp::Node
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

    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;


    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr startGameService;

    void startGame(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response);

};

}