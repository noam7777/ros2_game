#include <cstdio>
#include "game.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iron_dome_game::Game>());

  std::cout << "end of life"<<std::endl;
  rclcpp::shutdown();
  return 0;
}
