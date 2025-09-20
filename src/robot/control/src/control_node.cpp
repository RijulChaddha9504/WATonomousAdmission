#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <optional>
#include <chrono>

class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController() : Node("pure_pursuit_controller") {
        // Parameters
        lookahead_distance_ = 2.0;  // meters
        goal_tolerance_ = 0.1;      // meters
        linear_speed_ = 1.0;        // constant forward speed m/s
        stuck_time_threshold_ = 3.0; // seconds
        stuck_distance_threshold_ = 0.05; // meters

        // Subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&PurePursuitController::pathCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1));

        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PurePursuitController::controlLoop, this));

        last_position_time_ = this->now();
    }

private:
    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // Parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;

    // Stuck detection
    rclcpp::Time last_position_time_;
    geometry_msgs::msg::Point last_position_;
    double stuck_time_threshold_;
    double stuck_distance_threshold_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        current_path_ = msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_odom_ = msg;
    }

    void controlLoop() {
        if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
            return; // No path or odometry available
        }

        geometry_msgs::msg::Twist cmd_vel;

        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point) {
            // Stop robot if goal reached
            geometry_msgs::msg::Twist stop_msg;
            cmd_vel_pub_->publish(stop_msg);
            return;
        }

        cmd_vel = computeVelocity(*lookahead_point);

        // Stuck detection
        auto robot_pos = robot_odom_->pose.pose.position;
        double moved_distance = computeDistance(robot_pos, last_position_);
        rclcpp::Time now = this->now();

        if (moved_distance > stuck_distance_threshold_) {
            last_position_ = robot_pos;
            last_position_time_ = now;
        } else {
            double seconds_stuck = (now - last_position_time_).seconds();
            if (seconds_stuck > stuck_time_threshold_) {
                RCLCPP_WARN(this->get_logger(), "Robot seems stuck! Backing up.");
                cmd_vel.linear.x = -1.0;  // back up slowly
                cmd_vel.angular.z = 0.5;  // rotate slightly
                last_position_time_ = now; // reset timer after initiating backup
            }
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() {
        auto robot_pos = robot_odom_->pose.pose.position;

        for (auto &pose_stamped : current_path_->poses) {
            double dist = computeDistance(robot_pos, pose_stamped.pose.position);
            if (dist >= lookahead_distance_) {
                return pose_stamped;
            }
        }

        if (!current_path_->poses.empty()) {
            double final_dist = computeDistance(robot_pos, current_path_->poses.back().pose.position);
            if (final_dist < goal_tolerance_) {
                return std::nullopt; // Goal reached
            }
            return current_path_->poses.back();
        }

        return std::nullopt;
    }

    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
        geometry_msgs::msg::Twist cmd;

        auto robot_pos = robot_odom_->pose.pose.position;
        double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

        double dx = target.pose.position.x - robot_pos.x;
        double dy = target.pose.position.y - robot_pos.y;

        double distance_to_target = std::hypot(dx, dy);
        double target_angle = std::atan2(dy, dx);
        double angle_error = target_angle - robot_yaw;

        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        const double goal_distance_threshold = goal_tolerance_;
        const double goal_angle_threshold = 0.05;

        if (distance_to_target < goal_distance_threshold &&
            std::abs(angle_error) < goal_angle_threshold) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            return cmd;
        }

        // Linear speed taper
        double forward_component = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
        cmd.linear.x = std::copysign(linear_speed_, forward_component);

        const double slow_distance = 0.5;
        if (distance_to_target < slow_distance) {
            cmd.linear.x *= distance_to_target / slow_distance;
        }

        // Angular speed
        double k_ang = 1.0;
        double max_angular_speed = 1.0;
        cmd.angular.z = std::clamp(k_ang * angle_error, -max_angular_speed, max_angular_speed);

        return cmd;
    }

    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    double extractYaw(const geometry_msgs::msg::Quaternion &quat) {
        return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                          1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
