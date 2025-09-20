#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode() 
    : Node("map_memory_node"), last_x_(0.0), last_y_(0.0), distance_threshold_(0.0)
    {
        // Subscribers
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

        // Publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        // Timer for periodic updates (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MapMemoryNode::updateMap, this));

        RCLCPP_INFO(this->get_logger(), "MapMemoryNode initialized");
    }

private:
    // Subscribers and publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    double last_x_, last_y_;
    const double distance_threshold_;
    bool costmap_updated_ = false;
    bool should_update_map_ = false;

    // Callback: store latest costmap
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_costmap_ = *msg;
        costmap_updated_ = true;
    }

    // Callback: track robot movement
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));

        if (distance >= distance_threshold_) {
            last_x_ = x;
            last_y_ = y;
            should_update_map_ = true;
        }
    }

    // Timer-based map update
    void updateMap() {
        if (should_update_map_ && costmap_updated_) {
            integrateCostmap();
            map_pub_->publish(global_map_);
            should_update_map_ = false;
            RCLCPP_INFO(this->get_logger(), "Global map updated");
        }
    }

    // Simple linear fusion: overwrite known cells
    void integrateCostmap() {
        if (global_map_.data.empty()) {
            // Initialize global map from first costmap
            global_map_ = latest_costmap_;
            return;
        }

        // Merge latest_costmap_ into global_map_
        for (size_t i = 0; i < latest_costmap_.data.size(); ++i) {
            int8_t val = latest_costmap_.data[i];
            if (val >= 0) {  // Known cell (occupied or free)
                global_map_.data[i] = val;
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
