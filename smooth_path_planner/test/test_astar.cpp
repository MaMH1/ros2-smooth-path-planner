#include "smooth_path_planner/astar.hpp"
#include <cassert>
#include <iostream>

int main() {
  using namespace smooth_path_planner;
  Grid grid{5, 5, std::vector<bool>(25, false)};
  grid.blocked[2] = grid.blocked[7] = grid.blocked[12] = true;
  auto path = astar(grid, 0, 4);
  assert(!path.empty() && path.front() == 0 && path.back() == 4);
  for (int cell : path) assert(!grid.blocked[cell]);
  Grid corner{2, 2, {false, true, true, false}};
  assert(astar(corner, 0, 3).empty());
  assert(astar(grid, 0, 2).empty());
  assert(astar(grid, 0, 0).size() == 1);
  Grid wall{3, 3, {false,true,false,false,true,false,false,true,false}};
  assert(astar(wall, 0, 2).empty());
  std::cout << "A* tests passed\n";
}
