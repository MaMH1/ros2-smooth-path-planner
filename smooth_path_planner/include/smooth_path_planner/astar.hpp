#pragma once
#include <vector>
#include <cstdint>

namespace smooth_path_planner {
struct Grid {
  int width, height;
  std::vector<bool> blocked;
};
std::vector<int> astar(const Grid &grid, int start, int goal, const std::vector<int8_t> &costmap = {}, double weight = 0);
}  // namespace smooth_path_planner
