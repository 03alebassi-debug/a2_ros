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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

class MissionManager : public rclcpp::Node
{
public:
  MissionManager() : Node("mission_manager")
  {
    this->declare_parameter("exploration_duration", 300.0);
    this->declare_parameter("watchdog_enabled", true);
    this->declare_parameter("watchdog_stuck_timeout", 8.0);
    this->declare_parameter("watchdog_min_progress", 0.15);
    this->declare_parameter("watchdog_min_waypoint_distance", 0.8);
    this->declare_parameter("watchdog_goal_reached_dist", 0.5);
    this->declare_parameter("watchdog_stopped_linear_threshold", 0.03);
    this->declare_parameter("watchdog_stopped_angular_threshold", 0.03);
    this->declare_parameter("watchdog_blacklist_radius", 1.0);
    this->declare_parameter("watchdog_blacklist_ttl", 60.0);
    this->declare_parameter("watchdog_reset_cooldown", 3.0);
    this->declare_parameter("watchdog_escape_distance", 1.5);
    this->declare_parameter("watchdog_escape_hold_time", 5.0);

    exploration_duration_ = this->get_parameter("exploration_duration").as_double();
    watchdog_enabled_ = this->get_parameter("watchdog_enabled").as_bool();
    watchdog_stuck_timeout_ = this->get_parameter("watchdog_stuck_timeout").as_double();
    watchdog_min_progress_ = this->get_parameter("watchdog_min_progress").as_double();
    watchdog_min_waypoint_distance_ = this->get_parameter("watchdog_min_waypoint_distance").as_double();
    watchdog_goal_reached_dist_ = this->get_parameter("watchdog_goal_reached_dist").as_double();
    watchdog_stopped_linear_threshold_ = this->get_parameter("watchdog_stopped_linear_threshold").as_double();
    watchdog_stopped_angular_threshold_ = this->get_parameter("watchdog_stopped_angular_threshold").as_double();
    watchdog_blacklist_radius_ = this->get_parameter("watchdog_blacklist_radius").as_double();
    watchdog_blacklist_ttl_ = this->get_parameter("watchdog_blacklist_ttl").as_double();
    watchdog_reset_cooldown_ = this->get_parameter("watchdog_reset_cooldown").as_double();
    watchdog_escape_distance_ = this->get_parameter("watchdog_escape_distance").as_double();
    watchdog_escape_hold_time_ = this->get_parameter("watchdog_escape_hold_time").as_double();

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
        robot_position_ = msg->pose.pose.position;
        have_robot_position_ = true;
        update_progress_watchdog();
      });

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 5,
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        last_path_time_ = std::chrono::steady_clock::now();
        last_path_length_ = path_length(*msg);
        last_path_pose_count_ = msg->poses.size();
      });

    nav_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/nav_vel", 5,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_nav_vel_time_ = std::chrono::steady_clock::now();
        last_nav_speed_ = std::hypot(msg->twist.linear.x, msg->twist.linear.y);
        last_nav_yaw_rate_ = std::fabs(msg->twist.angular.z);
      });

    wp_pub_       = this->create_publisher<geometry_msgs::msg::PointStamped>("/selected_waypoint", 5);
    goal_pub_     = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 5);
    boundary_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/navigation_boundary", 5);
    reset_pub_    = this->create_publisher<std_msgs::msg::Empty>("/reset_waypoint", 1);

    timer_ = this->create_wall_timer(100ms, [this]() { loop(); });

    RCLCPP_INFO(this->get_logger(),
      "Mission manager ready — exploring for %.0f s, then returning home.",
      exploration_duration_);
  }

private:
  void loop()
  {
    prune_blacklist();

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
        publish_exploration_waypoint(*tare_waypoint_);
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

  static double point_distance_xy(const geometry_msgs::msg::Point & a,
                                  const geometry_msgs::msg::Point & b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  static double path_length(const nav_msgs::msg::Path & path)
  {
    double length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      length += std::hypot(a.x - b.x, a.y - b.y);
    }
    return length;
  }

  void prune_blacklist()
  {
    auto now = std::chrono::steady_clock::now();
    blacklist_.erase(
      std::remove_if(blacklist_.begin(), blacklist_.end(),
        [now](const BadWaypoint & bad) { return now >= bad.expires_at; }),
      blacklist_.end());
  }

  bool is_blacklisted(const geometry_msgs::msg::Point & point) const
  {
    for (const auto & bad : blacklist_) {
      if (point_distance_xy(point, bad.point) < watchdog_blacklist_radius_) {
        return true;
      }
    }
    return false;
  }

  void publish_escape_waypoint(const geometry_msgs::msg::Point & blocked_point)
  {
    if (!have_robot_position_) {
      return;
    }

    double dx = robot_position_.x - blocked_point.x;
    double dy = robot_position_.y - blocked_point.y;
    double norm = std::hypot(dx, dy);
    if (norm < 1e-3) {
      dx = 1.0;
      dy = 0.0;
      norm = 1.0;
    }

    geometry_msgs::msg::PointStamped escape;
    escape.header.stamp = this->now();
    escape.header.frame_id = "map";
    escape.point = robot_position_;
    escape.point.x += watchdog_escape_distance_ * dx / norm;
    escape.point.y += watchdog_escape_distance_ * dy / norm;
    escape_waypoint_ = escape;
    escape_until_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(watchdog_escape_hold_time_));
    have_escape_waypoint_ = true;
    wp_pub_->publish(escape_waypoint_);
  }

  bool publish_escape_if_active()
  {
    if (!have_escape_waypoint_) {
      return false;
    }
    if (std::chrono::steady_clock::now() >= escape_until_) {
      have_escape_waypoint_ = false;
      return false;
    }
    escape_waypoint_.header.stamp = this->now();
    wp_pub_->publish(escape_waypoint_);
    return true;
  }

  void request_tare_reset()
  {
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_reset_pub_).count() < watchdog_reset_cooldown_) {
      return;
    }
    reset_pub_->publish(std_msgs::msg::Empty{});
    last_reset_pub_ = now;
  }

  void blacklist_active_waypoint(const char * reason)
  {
    if (!have_active_waypoint_) {
      return;
    }

    BadWaypoint bad;
    bad.point = active_waypoint_.point;
    bad.expires_at = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(watchdog_blacklist_ttl_));
    blacklist_.push_back(bad);

    RCLCPP_WARN(this->get_logger(),
      "Exploration watchdog: blacklisting waypoint (%.2f, %.2f) for %.0f s: %s",
      bad.point.x, bad.point.y, watchdog_blacklist_ttl_, reason);

    have_active_waypoint_ = false;
    request_tare_reset();
    publish_escape_waypoint(bad.point);
  }

  void update_progress_watchdog()
  {
    if (!watchdog_enabled_ || !have_active_waypoint_ || !have_robot_position_) {
      return;
    }

    double distance_to_goal = point_distance_xy(robot_position_, active_waypoint_.point);
    if (distance_to_goal < watchdog_goal_reached_dist_) {
      have_active_waypoint_ = false;
      return;
    }

    if (closest_active_distance_ - distance_to_goal > watchdog_min_progress_) {
      closest_active_distance_ = distance_to_goal;
      last_progress_time_ = std::chrono::steady_clock::now();
    }
  }

  bool watchdog_should_blacklist(std::string & reason)
  {
    if (!watchdog_enabled_ || !have_active_waypoint_ || !have_robot_position_) {
      return false;
    }

    auto now = std::chrono::steady_clock::now();
    double distance_to_goal = point_distance_xy(robot_position_, active_waypoint_.point);
    if (distance_to_goal < watchdog_goal_reached_dist_ ||
        distance_to_goal < watchdog_min_waypoint_distance_) {
      return false;
    }

    bool recently_commanded = std::chrono::duration<double>(now - last_nav_vel_time_).count() < 1.0;
    bool effectively_stopped =
      !recently_commanded ||
      (last_nav_speed_ < watchdog_stopped_linear_threshold_ &&
       last_nav_yaw_rate_ < watchdog_stopped_angular_threshold_);
    if (effectively_stopped &&
        std::chrono::duration<double>(now - last_progress_time_).count() > watchdog_stuck_timeout_) {
      reason = "no linear or angular command while far from waypoint";
      return true;
    }

    return false;
  }

  void start_tracking_waypoint(const geometry_msgs::msg::PointStamped & waypoint)
  {
    active_waypoint_ = waypoint;
    active_waypoint_time_ = std::chrono::steady_clock::now();
    last_progress_time_ = active_waypoint_time_;
    closest_active_distance_ = have_robot_position_
      ? point_distance_xy(robot_position_, waypoint.point)
      : std::numeric_limits<double>::infinity();
    have_active_waypoint_ = true;
  }

  void publish_exploration_waypoint(const geometry_msgs::msg::PointStamped & waypoint)
  {
    if (!watchdog_enabled_) {
      wp_pub_->publish(waypoint);
      return;
    }

    if (publish_escape_if_active()) {
      return;
    }

    if (is_blacklisted(waypoint.point)) {
      publish_escape_waypoint(waypoint.point);
      request_tare_reset();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Exploration watchdog: rejecting blacklisted TARE waypoint (%.2f, %.2f), publishing escape waypoint",
        waypoint.point.x, waypoint.point.y);
      return;
    }

    if (!have_active_waypoint_ ||
        point_distance_xy(active_waypoint_.point, waypoint.point) > 0.3) {
      start_tracking_waypoint(waypoint);
    }

    std::string reason;
    if (watchdog_should_blacklist(reason)) {
      blacklist_active_waypoint(reason.c_str());
      return;
    }

    wp_pub_->publish(waypoint);
  }

  bool   home_set_  = false;
  bool   returning_ = false;
  double exploration_duration_;

  bool watchdog_enabled_ = true;
  double watchdog_stuck_timeout_ = 8.0;
  double watchdog_min_progress_ = 0.15;
  double watchdog_min_waypoint_distance_ = 0.8;
  double watchdog_goal_reached_dist_ = 0.5;
  double watchdog_stopped_linear_threshold_ = 0.03;
  double watchdog_stopped_angular_threshold_ = 0.03;
  double watchdog_blacklist_radius_ = 1.0;
  double watchdog_blacklist_ttl_ = 60.0;
  double watchdog_reset_cooldown_ = 3.0;
  double watchdog_escape_distance_ = 1.5;
  double watchdog_escape_hold_time_ = 5.0;

  geometry_msgs::msg::Point home_;
  geometry_msgs::msg::Point robot_position_;
  bool have_robot_position_ = false;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_goal_pub_;
  std::chrono::steady_clock::time_point last_status_print_;
  std::chrono::steady_clock::time_point last_path_time_{};
  std::chrono::steady_clock::time_point last_nav_vel_time_{};
  std::chrono::steady_clock::time_point last_reset_pub_{};
  std::chrono::steady_clock::time_point active_waypoint_time_{};
  std::chrono::steady_clock::time_point last_progress_time_{};
  std::chrono::steady_clock::time_point escape_until_{};

  struct BadWaypoint
  {
    geometry_msgs::msg::Point point;
    std::chrono::steady_clock::time_point expires_at;
  };
  std::vector<BadWaypoint> blacklist_;

  geometry_msgs::msg::PointStamped active_waypoint_;
  geometry_msgs::msg::PointStamped escape_waypoint_;
  bool have_active_waypoint_ = false;
  bool have_escape_waypoint_ = false;
  double closest_active_distance_ = std::numeric_limits<double>::infinity();
  double last_path_length_ = 0.0;
  size_t last_path_pose_count_ = 0;
  double last_nav_speed_ = 0.0;
  double last_nav_yaw_rate_ = 0.0;

  geometry_msgs::msg::PointStamped::SharedPtr   tare_waypoint_;
  geometry_msgs::msg::PointStamped::SharedPtr   far_waypoint_;
  geometry_msgs::msg::PolygonStamped::SharedPtr far_boundary_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr   tare_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr   far_wp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr far_boundary_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr   nav_vel_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr      wp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr      goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr    boundary_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                  reset_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
