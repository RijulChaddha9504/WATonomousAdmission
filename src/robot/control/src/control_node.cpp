#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <chrono>

class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController() : Node("pure_pursuit_controller") {
        lookahead_distance_ = 2.0;
        goal_tolerance_ = 0.1;
        linear_speed_ = 1.0;
        stuck_time_threshold_ = 3.0;
        stuck_distance_threshold_ = 0.05;

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&PurePursuitController::pathCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PurePursuitController::controlLoop, this));

        last_position_time_ = this->now();
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    double lookahead_distance_, goal_tolerance_, linear_speed_;
    double stuck_time_threshold_, stuck_distance_threshold_;
    rclcpp::Time last_position_time_;
    geometry_msgs::msg::Point last_position_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; }

    void controlLoop() {
        if (!current_path_ || current_path_->poses.empty() || !robot_odom_){
            return;
        }

        auto lookahead = getLookaheadPoint();
        geometry_msgs::msg::Twist cmd;

        if (!lookahead) {
            cmd_vel_pub_->publish(cmd);
            return;
        }

        cmd = computeVelocity(*lookahead);

        auto pos = robot_odom_->pose.pose.position;
        double moved = distance(pos, last_position_);
        auto now = this->now();

        if (moved > stuck_distance_threshold_) {
            last_position_ = pos;
            last_position_time_ = now;
        } else if ((now - last_position_time_).seconds() > stuck_time_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Stuck detected. Backing up.");
            cmd.linear.x = -1.0;
            cmd.angular.z = 0.5;
            last_position_time_ = now;
        }

        cmd_vel_pub_->publish(cmd);
    }

    geometry_msgs::msg::PoseStamped* getLookaheadPoint() {
        auto pos = robot_odom_->pose.pose.position;
        for (auto &p : current_path_->poses) {
            if (distance(pos, p.pose.position) >= lookahead_distance_){
                return &p;
            }
        }
        if (!current_path_->poses.empty()) {
            auto &goal = current_path_->poses.back();
            if (distance(pos, goal.pose.position) < goal_tolerance_){
                return nullptr;
            }
            return &current_path_->poses.back();
        }
        return nullptr;
    }

    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
        geometry_msgs::msg::Twist cmd;
        auto pos = robot_odom_->pose.pose.position;
        double yaw = getYaw(robot_odom_->pose.pose.orientation);

        double dx = target.pose.position.x - pos.x;
        double dy = target.pose.position.y - pos.y;
        double dist = std::hypot(dx, dy);
        double target_angle = std::atan2(dy, dx);
        double error = normalizeAngle(target_angle - yaw);

        if (dist < goal_tolerance_ && std::abs(error) < 0.05){
            return cmd;
        }

        double forward = dx * std::cos(yaw) + dy * std::sin(yaw);
        cmd.linear.x = std::copysign(linear_speed_, forward);

        if (dist < 0.5){
            cmd.linear.x *= dist / 0.5;
        }

        cmd.angular.z = std::clamp(error, -1.0, 1.0);
        return cmd;
    }

    double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    double getYaw(const geometry_msgs::msg::Quaternion &q) {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    double normalizeAngle(double a) {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}
