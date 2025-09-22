// planner_node_simple.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <queue>
#include <cmath>
#include <limits>

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode()
  : Node("planner_node_simple"),
    goal_received_(false),
    map_received_(false),
    last_plan_time_(this->now())
  {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 20, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PlannerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "PlannerNode (simple) started");
  }

private:
  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose robot_pose_;
  bool goal_received_;
  bool map_received_;

  // Params
  double goal_tolerance_ = 0.5;
  double obstacle_threshold_ = 50.0; // occupancy >= threshold => obstacle
  rclcpp::Time last_plan_time_;
  rclcpp::Duration replan_timeout_ = rclcpp::Duration::from_seconds(10.0);

  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Map received: %u x %u",
                current_map_.info.width, current_map_.info.height);
    if (goal_received_) planPath();
  }

  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
    planPath();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
  }

  void timerCallback() {
    if (!goal_received_) return;
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached.");
      goal_received_ = false;
      return;
    }
    if ((this->now() - last_plan_time_) >= replan_timeout_) {
      RCLCPP_INFO(this->get_logger(), "Replan timeout -> replanning");
      planPath();
    }
  }

  // Helpers (no std::optional used)
  bool goalReached() {
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx*dx + dy*dy) < goal_tolerance_;
  }

  // Convert world coords to grid index (returns false if out of bounds or no map)
  bool worldToIndex(double wx, double wy, int &idx_out) {
    if (!map_received_) return false;
    double ox = current_map_.info.origin.position.x;
    double oy = current_map_.info.origin.position.y;
    double res = current_map_.info.resolution;
    int width = static_cast<int>(current_map_.info.width);
    int height = static_cast<int>(current_map_.info.height);

    int ix = static_cast<int>(std::floor((wx - ox) / res));
    int iy = static_cast<int>(std::floor((wy - oy) / res));
    if (ix < 0 || iy < 0 || ix >= width || iy >= height) return false;
    idx_out = iy * width + ix;
    return true;
  }

  geometry_msgs::msg::Point indexToWorld(int idx) {
    geometry_msgs::msg::Point p;
    int width = static_cast<int>(current_map_.info.width);
    int ix = idx % width;
    int iy = idx / width;
    double ox = current_map_.info.origin.position.x;
    double oy = current_map_.info.origin.position.y;
    double res = current_map_.info.resolution;
    p.x = ox + (ix + 0.5) * res;
    p.y = oy + (iy + 0.5) * res;
    p.z = 0.0;
    return p;
  }

  bool isCellFreeIndex(int idx) {
    if (!map_received_) return false;
    int w = static_cast<int>(current_map_.info.width);
    int h = static_cast<int>(current_map_.info.height);
    if (idx < 0 || idx >= w*h) return false;
    int8_t val = current_map_.data[idx];
    if (val < 0) return true;             // unknown = treat as free
    return (val < obstacle_threshold_);
  }

  void planPath() {
    if (!goal_received_) {
      RCLCPP_WARN(this->get_logger(), "No goal to plan to.");
      return;
    }
    if (!map_received_) {
      RCLCPP_WARN(this->get_logger(), "No map available.");
      return;
    }

    int width = static_cast<int>(current_map_.info.width);
    int height = static_cast<int>(current_map_.info.height);
    if (width <= 0 || height <= 0) {
      RCLCPP_WARN(this->get_logger(), "Map has invalid size.");
      return;
    }
    int total = width * height;

    int start_idx = -1, goal_idx = -1;
    if (!worldToIndex(robot_pose_.position.x, robot_pose_.position.y, start_idx)) {
      RCLCPP_WARN(this->get_logger(), "Start out of bounds.");
      return;
    }
    if (!worldToIndex(goal_.point.x, goal_.point.y, goal_idx)) {
      RCLCPP_WARN(this->get_logger(), "Goal out of bounds.");
      return;
    }

    if (!isCellFreeIndex(start_idx)) {
      RCLCPP_WARN(this->get_logger(), "Start cell blocked.");
      return;
    }
    if (!isCellFreeIndex(goal_idx)) {
      RCLCPP_WARN(this->get_logger(), "Goal cell blocked.");
      return;
    }

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> g_score(total, INF);
    std::vector<int> came_from(total, -1);
    std::vector<char> closed(total, 0);

    auto heuristic = [&](int a, int b) {
      int wx = a % width, wy = a / width;
      int gx = b % width, gy = b / width;
      double dx = double(wx - gx);
      double dy = double(wy - gy);
      return std::sqrt(dx*dx + dy*dy);
    };

    // min-heap (f_score, idx)
    std::priority_queue<std::pair<double,int>,
                        std::vector<std::pair<double,int>>,
                        std::greater<std::pair<double,int>>> open;

    g_score[start_idx] = 0.0;
    open.emplace(heuristic(start_idx, goal_idx), start_idx);

    // neighbor offsets for 8-connected grid (dx,dy)
    const int dxs[8] = {1,-1,0,0,1,1,-1,-1};
    const int dys[8] = {0,0,1,-1,1,-1,1,-1};

    bool found = false;
    int found_idx = -1;

    while (!open.empty()) {
      auto top = open.top(); open.pop();
      int current = top.second;
      if (closed[current]) continue;
      closed[current] = 1;
      if (current == goal_idx) { found = true; found_idx = current; break; }

      int cx = current % width;
      int cy = current / width;

      for (int i = 0; i < 8; ++i) {
        int nx = cx + dxs[i];
        int ny = cy + dys[i];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
        int nidx = ny * width + nx;
        if (!isCellFreeIndex(nidx)) continue;

        double move_cost = (dxs[i] != 0 && dys[i] != 0) ? std::sqrt(2.0) : 1.0;
        double tentative = g_score[current] + move_cost;
        if (tentative < g_score[nidx]) {
          g_score[nidx] = tentative;
          came_from[nidx] = current;
          double f = tentative + heuristic(nidx, goal_idx);
          open.emplace(f, nidx);
        }
      }
    }

    // Build and publish result
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = current_map_.header.frame_id;

    if (!found) {
      RCLCPP_WARN(this->get_logger(), "A*: no path found");
      path_pub_->publish(path_msg); // empty path
      last_plan_time_ = this->now();
      return;
    }

    std::vector<int> rev_path;
    int cur = found_idx;
    while (cur != -1 && cur != start_idx) {
      rev_path.push_back(cur);
      cur = came_from[cur];
    }
    if (cur == start_idx) rev_path.push_back(start_idx);
    std::reverse(rev_path.begin(), rev_path.end());

    for (int idx : rev_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position = indexToWorld(idx);
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }

    path_pub_->publish(path_msg);
    last_plan_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path_msg.poses.size());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
