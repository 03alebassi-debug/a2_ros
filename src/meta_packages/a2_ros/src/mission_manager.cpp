/**
 * Mission Manager
 *
 * Muxes between TARE (exploration) and FAR (return-home) waypoints based on a
 * wall-clock timer set via the `exploration_duration` parameter.
 *
 * Topic flow:
 *   Exploration phase (t < exploration_duration):
 *     /way_point (TARE) ──► /selected_waypoint ──► localPlanner
 *
 *   Return phase (t >= exploration_duration):
 *     /goal_point  ──► FAR planner  (home position, repeated every 1 s)
 *     /far_waypoint (FAR output) ──► /selected_waypoint ──► localPlanner
 *     /far_navigation_boundary   ──► /navigation_boundary ──► localPlanner
 */

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class MissionManager : public rclcpp::Node
{
public:
  MissionManager() : Node("mission_manager")
  {
    this->declare_parameter("exploration_duration", 300.0);
    exploration_duration_ = this->get_parameter("exploration_duration").as_double();

    start_time_       = std::chrono::steady_clock::now();
    last_goal_pub_    = start_time_ - std::chrono::seconds(10);
    last_status_print_ = start_time_ - std::chrono::seconds(10);

    tare_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/way_point", 5,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        tare_waypoint_ = msg;
      });

    far_wp_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/far_waypoint", 5,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        far_waypoint_ = msg;
      });

    far_boundary_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/far_navigation_boundary", 5,
      [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        far_boundary_ = msg;
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 5,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!home_set_) {
          home_     = msg->pose.pose.position;
          home_set_ = true;
          RCLCPP_INFO(this->get_logger(), "Home recorded: (%.2f, %.2f, %.2f)",
            home_.x, home_.y, home_.z);
        }
      });

    wp_pub_       = this->create_publisher<geometry_msgs::msg::PointStamped>("/selected_waypoint", 5);
    goal_pub_     = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 5);
    boundary_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/navigation_boundary", 5);

    timer_ = this->create_wall_timer(100ms, [this]() { loop(); });

    RCLCPP_INFO(this->get_logger(),
      "Mission manager ready — exploring for %.0f s, then returning home.",
      exploration_duration_);
  }

private:
  void loop()
  {
    double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();

    if (elapsed < exploration_duration_) {
      if (returning_) {
        returning_ = false;
      }
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - last_status_print_).count() >= 10.0) {
        RCLCPP_INFO(this->get_logger(),
          "Exploring — %.0f s remaining before returning home.",
          exploration_duration_ - elapsed);
        last_status_print_ = now;
      }
      if (tare_waypoint_) {
        wp_pub_->publish(*tare_waypoint_);
      }
    } else {
      if (!returning_) {
        returning_ = true;
        RCLCPP_INFO(this->get_logger(),
          "=== Exploration complete (%.0f s elapsed) — returning to home (%.2f, %.2f) ===",
          elapsed, home_.x, home_.y);
      }

      auto now = std::chrono::steady_clock::now();
      if (home_set_ &&
          std::chrono::duration<double>(now - last_goal_pub_).count() > 1.0)
      {
        geometry_msgs::msg::PointStamped goal;
        auto ns = this->now().nanoseconds();
        goal.header.stamp.sec    = static_cast<int32_t>(ns / 1'000'000'000LL);
        goal.header.stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
        goal.header.frame_id = "map";
        goal.point           = home_;
        goal_pub_->publish(goal);
        last_goal_pub_ = now;
      }

      if (far_waypoint_) {
        wp_pub_->publish(*far_waypoint_);
      }

      if (far_boundary_) {
        boundary_pub_->publish(*far_boundary_);
      }
    }
  }

  bool   home_set_  = false;
  bool   returning_ = false;
  double exploration_duration_;

  geometry_msgs::msg::Point home_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_goal_pub_;
  std::chrono::steady_clock::time_point last_status_print_;

  geometry_msgs::msg::PointStamped::SharedPtr   tare_waypoint_;
  geometry_msgs::msg::PointStamped::SharedPtr   far_waypoint_;
  geometry_msgs::msg::PolygonStamped::SharedPtr far_boundary_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr   tare_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr   far_wp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr far_boundary_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr      wp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr      goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr    boundary_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
