#include "smooth_path_planner/astar.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace smooth_path_planner {//入口与启发式
std::vector<int> astar(const Grid &grid, int start, int goal, const std::vector<int8_t> &costmap, double weight) {//定义A*搜索函数，grid 是只读引用提供禁行信息
  if (grid.blocked[start] || grid.blocked[goal]) return {};
  const int w = grid.width;
  auto heuristic = [w, goal](int cell) {//估计代价函数
    const int dx = std::abs(cell % w - goal % w);
    const int dy = std::abs(cell / w - goal / w);//计算当前格 cell 和目标格 goal 在横向、纵向各相差几格
    return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);//斜走加直走
  };//结束Lambda定义
  using Entry = std::pair<double, int>;//队列与回溯//把预计总代价和格子编号打包成 Entry
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;//建立候选队列 open，greater 让较小的总代价先出队
  std::vector<double> cost(grid.blocked.size(), std::numeric_limits<double>::infinity());//每格分配一个 cost，记录已知从起点到该格的最低代价
  std::vector<int> parent(grid.blocked.size(), -1);//每格保存前驱编号，-1 表示暂未记录
  std::vector<bool> closed(grid.blocked.size(), false);//每格分配一个处理标记，初始 false 表示还没完成扩展
  cost[start] = 0;
  open.emplace(heuristic(start), start);//起点入队
  while (!open.empty()) {
    const int current = open.top().second;//读出队首条目中的格子编号
    open.pop();//移除队首，编号已复制到 current。
    if (closed[current]) continue;//一个格子可能以不同代价多次入队。若它已经处理过，就跳到下一轮
    if (current == goal) {//如果这次取出的格子就是终点，开始还原路径
      std::vector<int> path;
      for (int cell = goal; cell != -1; cell = parent[cell]) path.push_back(cell);//从终点沿 parent 倒着记录格子，直到前驱为 -1
      std::reverse(path.begin(), path.end());
      return path;
    }
    closed[current] = true;//当前格标记为已展开
    const int x = current % w, y = current / w;
    for (int dy = -1; dy <= 1; ++dy) {//遍历八个方向
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const int nx = x + dx, ny = y + dy;//计算相邻格的列和行
        if (nx < 0 || nx >= w || ny < 0 || ny >= grid.height) continue;//邻格越界时跳过
        const int next = ny * w + nx;
        if (grid.blocked[next]) continue;//邻格是障碍时跳过
        // 斜向移动时检查两个紧邻侧格，两侧也必须可通行，避免穿过墙角。
        if (dx && dy && (grid.blocked[y * w + nx] || grid.blocked[ny * w + x])) continue;
        const double step = dx && dy ? std::sqrt(2.0) : 1.0;//基础代价
        const double penalty = costmap.empty() ? 0.0 : weight * costmap[next] / 100.0;//若 costmap 为空，惩罚取零，否则用目标邻格的值乘权重再除以 100
        const double candidate = cost[current] + step * (1.0 + penalty);
        if (candidate >= cost[next]) continue;
        cost[next] = candidate;
        parent[next] = current;
        open.emplace(candidate + heuristic(next), next);
      }
    }
  }
  return {};
}
}  // namespace smooth_path_planner
