#include <cstdio>
#include "pkg_iron_dome/physics.hpp"
#include "game.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  iron_dome_game::Game game;
  game.play();
  std::cout << "end of life"<<std::endl;
  return 0;
}
