#include "smooth_path_planner/astar.hpp"
#include <rclcpp/rclcpp.hpp>  //ROS2的C++接口
#include <nav_msgs/msg/occupancy_grid.hpp> //栅格地图消息
#include <nav_msgs/msg/path.hpp>  //路径消息，包括消息头（坐标系名称和时间戳）和一串位姿
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>//带消息头和协方差的位姿
#include <geometry_msgs/msg/pose_stamped.hpp>//带消息头的位姿
#include <algorithm>
#include <cmath>
#include <optional>

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Path;


class Planner : public rclcpp::Node {
 public:
  Planner() : Node("smooth_planner") {
    weight_ = declare_parameter("cost_weight", 3.0);
    raw_pub_ = create_publisher<Path>("/plan_raw", rclcpp::QoS(1).transient_local());
    path_pub_ = create_publisher<Path>("/plan", rclcpp::QoS(1).transient_local());//创建/plan发布者，深度1表示只保留最近一条历史消息
    map_sub_ = create_subscription<OccupancyGrid>("/costmap/costmap", rclcpp::QoS(1).transient_local(),//创建 /map 订阅
      [this](OccupancyGrid::ConstSharedPtr map) {//每次收到/map消息，执行回调函数
        const bool first_map = !map_;
        if (map_ && map_->data == map->data && map_->info == map->info) return;
        map_ = map;
        build_grid(); 
        if (first_map) RCLCPP_INFO(get_logger(), "Costmap ready. Select start and goal in RViz.");//输出提示日志
        plan();
      });
    start_sub_ = create_subscription<PoseWithCovarianceStamped>("/initialpose", 10,//接收带协方差的起点位姿，队列深度为10
      [this](PoseWithCovarianceStamped::ConstSharedPtr msg) {//收到起点时执行回调，msg指向只读消息
        PoseStamped pose;//创建一个普通的带消息头的位姿，用于存储起点
        pose.header = msg->header;//复制消息头，保留坐标系名称和时间戳。
        pose.pose = msg->pose.pose;//外层 pose 包含位姿与协方差，内层 pose 才是位置和朝向。这里只复制内层位姿。
        start_ = pose;//记录已选择起点
        RCLCPP_INFO(get_logger(), "Start selected.");
        plan();
      });//起点设计接收带协方差的位姿，但后面路径搜索没有用到协方差
    goal_sub_ = create_subscription<PoseStamped>("/goal_pose", 10,//定义收到新目标时执行的回调
      [this](PoseStamped::ConstSharedPtr msg) {
        goal_ = *msg;
        RCLCPP_INFO(get_logger(), "Goal selected.");
        plan();
      });
  }
 private:
  void build_grid() {
    const auto &info = map_->info;//声明只读引用，相当于别名
    grid_ = {static_cast<int>(info.width), static_cast<int>(info.height),//重建grid_，将地图宽和高转换为整数格数
             std::vector<bool>(map_->data.size(), false)};//将地图先全设为可通行
    const auto &q = info.origin.orientation;//取得地图栅格原点的朝向四元数 q
    yaw_ = std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));//从四元数计算绕 z 轴的偏航角 yaw_，单位为弧度。它表示栅格轴相对于地图消息坐标轴转了多少。
    for (size_t i = 0; i < map_->data.size(); ++i)
      grid_.blocked[i] = map_->data[i] < 0 || map_->data[i] >= 99;
  }

  // 在地图坐标下检查短线段覆盖的栅格。用包围框保守处理擦角。
  bool clear_segment(const PoseStamped &a, const PoseStamped &b) const {
    const int from = cell(a), to = cell(b);//把两端点换算成所在格子的编号
    if (from < 0 || to < 0) return false;
    const int w = grid_.width;
    for (int y = std::min(from/w, to/w); y <= std::max(from/w, to/w); ++y)
      for (int x = std::min(from%w, to%w); x <= std::max(from%w, to%w); ++x)
        if (grid_.blocked[y*w+x]) return false;
    return true;
  }

  PoseStamped blend(const PoseStamped &a, const PoseStamped &b, double t) const {
    auto p = a;
    p.pose.position.x += t * (b.pose.position.x - a.pose.position.x);
    p.pose.position.y += t * (b.pose.position.y - a.pose.position.y);
    return p;
  }

  void smooth(Path &path) const {//定义平滑函数，用短斜线切掉尖角
    // 逐轮切角，每段保持在原栅格尺度内，失败时保留该拐角。
    for (int pass = 0; pass < 3 && path.poses.size() > 2; ++pass) {//最多平滑3轮
      const auto &points = path.poses;
      std::vector<PoseStamped> rounded{points.front()};
      for (size_t i = 1; i + 1 < points.size(); ++i) {
        auto entry = blend(points[i], points[i-1], 0.25);//当前路径点 points[i] 朝前一个点 points[i-1] 移动两点间距离的 25%，得到切角的入口 entry
        auto exit = blend(points[i], points[i+1], 0.25);
        if (clear_segment(rounded.back(), entry) &&
            clear_segment(entry, exit) && clear_segment(exit, points[i+1])) {
          rounded.push_back(entry);
          rounded.push_back(exit);
        } else {
          rounded.push_back(points[i]);
        }
      }
      rounded.push_back(points.back());
      path.poses = std::move(rounded);
    }
  }

  int cell(const PoseStamped &pose) const {//定义位姿到格子编号的转换函数
    const double dx = pose.pose.position.x - map_->info.origin.position.x;
    const double dy = pose.pose.position.y - map_->info.origin.position.y;
    const int x = std::floor((std::cos(yaw_)*dx + std::sin(yaw_)*dy) / map_->info.resolution);
    const int y = std::floor((-std::sin(yaw_)*dx + std::cos(yaw_)*dy) / map_->info.resolution);
    if (x < 0 || x >= grid_.width || y < 0 || y >= grid_.height) return -1;
    return y * grid_.width + x;
  }

  void plan() {
    if (!map_ || !start_ || !goal_) return;
    Path path;//新建一条空路径，稍后向其中添加路径点
    path.header.frame_id = map_->header.frame_id;
    path.header.stamp = now();
    // 本演示统一使用地图坐标系，不引入 TF 转换。
    if (start_->header.frame_id != path.header.frame_id || goal_->header.frame_id != path.header.frame_id) {
      RCLCPP_WARN(get_logger(), "Select points in the map frame.");
      raw_pub_->publish(path);
      path_pub_->publish(path);
      return;
    }//在地图坐标系下选点
    const int start = cell(*start_), goal = cell(*goal_);//取出两个 optional 内保存的位姿，调用 cell 分别得到起点和终点的格子编号
    if (start < 0 || goal < 0 || grid_.blocked[start] || grid_.blocked[goal]) {
      RCLCPP_WARN(get_logger(), "Start/goal is outside free space (including safety margin).");
      raw_pub_->publish(path);
      path_pub_->publish(path);
      return;
    }
    const auto route = smooth_path_planner::astar(grid_, start, goal, map_->data, weight_);//调用astar规划路径
    for (int index : route) {//将编号还原成路径点
      PoseStamped pose;
      pose.header = path.header;
      const double x = (index % grid_.width + 0.5) * map_->info.resolution;
      const double y = (index / grid_.width + 0.5) * map_->info.resolution;
      pose.pose.position.x = map_->info.origin.position.x + std::cos(yaw_)*x - std::sin(yaw_)*y;
      pose.pose.position.y = map_->info.origin.position.y + std::sin(yaw_)*x + std::cos(yaw_)*y;
      pose.pose.orientation.w = 1;
      path.poses.push_back(pose);
    }
    raw_pub_->publish(path);
    smooth(path);
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto &a = path.poses[i].pose.position;
      const auto &b = path.poses[i+1].pose.position;
      const double angle = std::atan2(b.y-a.y, b.x-a.x);
      path.poses[i].pose.orientation.z = std::sin(angle/2);
      path.poses[i].pose.orientation.w = std::cos(angle/2);
    }
    if (!path.poses.empty()) path.poses.back().pose.orientation = goal_->pose.orientation;
    path_pub_->publish(path);
    if (route.empty()) RCLCPP_WARN(get_logger(), "No route found.");
    else RCLCPP_INFO(get_logger(), "Published path: %zu grid cells.", route.size());
  }

  double weight_, yaw_ = 0;
  smooth_path_planner::Grid grid_;
  OccupancyGrid::ConstSharedPtr map_;
  std::optional<PoseStamped> start_, goal_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_, raw_pub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
}
