#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode() : Node("costmap_node") {
        // Basic settings
        resolution_ = 0.1;   // meters per grid cell
        width_ = 300;        // number of cells in x
        height_ = 300;       // number of cells in y
        origin_x_ = -15.0;    // origin in meters
        origin_y_ = -15.0;
        inflation_radius_ = 3.0; 
        max_cost_ = 100;

        // Allocate grid
        costmap_.resize(height_, std::vector<int>(width_, 0));

        // Subscribe to LiDAR
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", 10,
            std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

        // Publisher for costmap
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
    }

private:
    // Reset map to all zeros
    void initializeCostmap() {
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                costmap_[y][x] = 0;
            }
        }
    }

    // Convert polar (range, angle) to grid indices
    void convertToGrid(double range, double angle, int &x_idx, int &y_idx) {
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);

        x_idx = static_cast<int>((x - origin_x_) / resolution_);
        y_idx = static_cast<int>((y - origin_y_) / resolution_);
    }

    // Mark a cell as occupied
    void markObstacle(int x_idx, int y_idx) {
        if (x_idx >= 0 && x_idx < width_ && y_idx >= 0 && y_idx < height_) {
            costmap_[y_idx][x_idx] = max_cost_;
        }
    }

    // Inflate obstacles with a simple distance check
    void inflateObstacles() {
        std::vector<std::vector<int>> inflated = costmap_;

        int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                if (costmap_[y][x] == max_cost_) {
                    for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
                        for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

                            double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
                            if (dist <= inflation_radius_) {
                                int cost = (int)(max_cost_ * (1.0 - dist / inflation_radius_));
                                inflated[ny][nx] = std::max(inflated[ny][nx], cost);
                            }
                        }
                    }
                }
            }
        }
        costmap_ = inflated;
    }

    // Publish grid as OccupancyGrid
    void publishCostmap() {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = this->now();
        grid.header.frame_id = "map";

        grid.info.resolution = resolution_;
        grid.info.width = width_;
        grid.info.height = height_;
        grid.info.origin.position.x = origin_x_;
        grid.info.origin.position.y = origin_y_;
        grid.info.origin.orientation.w = 1.0;

        grid.data.resize(width_ * height_);
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                grid.data[y * width_ + x] = costmap_[y][x];
            }
        }

        costmap_pub_->publish(grid);
    }

    // Main callback for LiDAR
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        initializeCostmap();

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            double angle = scan->angle_min + i * scan->angle_increment;
            double range = scan->ranges[i];

            if (range > scan->range_min && range < scan->range_max) {
                int gx, gy;
                convertToGrid(range, angle, gx, gy);
                markObstacle(gx, gy);
            }
        }

        inflateObstacles();
        publishCostmap();
    }

    // --- Members ---
    double resolution_;
    int width_, height_;
    double origin_x_, origin_y_;
    double inflation_radius_;
    int max_cost_;
    std::vector<std::vector<int>> costmap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

// Main entry
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
