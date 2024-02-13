#include <cstdio>
#include "pkg_iron_dome/physics.hpp"
#include "game.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iron_dome_game::Game>());

  // game.play();
  std::cout << "end of life"<<std::endl;
  rclcpp::shutdown();
  return 0;
}
