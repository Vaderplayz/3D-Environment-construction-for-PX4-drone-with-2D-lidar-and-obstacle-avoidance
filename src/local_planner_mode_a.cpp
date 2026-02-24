// Last modified: 2026-02-25 00:10:00 +07
// Added: escape-aware final safety gating (uses future clearance trend, not t=0 lockout) to prevent pole-stare freezes
// Removed: overly conservative final gate behavior that blocked all commands when current clearance started slightly below gate

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

using namespace std::chrono_literals;

namespace {
inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

struct ObPoint { double x, y; };

static double yaw_from_odom(const nav_msgs::msg::Odometry &odom) {
  tf2::Quaternion q(
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  return y;
}

inline void rot_body_to_world(double yaw, double vx_b, double vy_b, double &vx_w, double &vy_w) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx_w = c * vx_b - s * vy_b;
  vy_w = s * vx_b + c * vy_b;
}
} // namespace

class DwaLocalPlanner : public rclcpp::Node {
public:
  DwaLocalPlanner() : Node("dwa_local_planner_skeleton") {
    // -------- Parameters (keep simple first) --------
    v_max_  = declare_parameter<double>("v_max", 2.5);
    vy_max_ = declare_parameter<double>("vy_max", 2.0);
    w_max_  = declare_parameter<double>("w_max", 1.0);
    vz_max_ = declare_parameter<double>("vz_max", 1.0);

    ax_max_ = declare_parameter<double>("ax_max", 1.0);
    ay_max_ = declare_parameter<double>("ay_max", 1.0);
    aw_max_ = declare_parameter<double>("aw_max", 1.2);

    control_dt_ = declare_parameter<double>("control_dt", 0.05);
    sim_dt_     = declare_parameter<double>("sim_dt", 0.10);
    horizon_    = declare_parameter<double>("horizon_sec", 1.6);

    nx_ = declare_parameter<int>("nx_samples", 9);
    ny_ = declare_parameter<int>("ny_samples", 7);
    nw_ = declare_parameter<int>("nw_samples", 11);

    max_use_range_ = declare_parameter<double>("max_use_range", 12.0);
    scan_stride_   = declare_parameter<int>("scan_stride", 2);
    safety_radius_ = declare_parameter<double>("safety_radius", 0.80);
    collision_radius_ = declare_parameter<double>("collision_radius", 0.32);
    collision_from_safety_scale_ =
        declare_parameter<double>("collision_from_safety_scale", 0.85);
    hard_clearance_margin_ = declare_parameter<double>("hard_clearance_margin", 0.05);
    final_cmd_clearance_margin_ = declare_parameter<double>("final_cmd_clearance_margin", 0.08);
    final_cmd_escape_relax_ = declare_parameter<double>("final_cmd_escape_relax", 0.20);
    final_cmd_escape_trend_eps_ = declare_parameter<double>("final_cmd_escape_trend_eps", 0.03);
    obs_range_     = declare_parameter<double>("obstacle_cloud_range", 8.0);
    fallback_strafe_speed_ = declare_parameter<double>("fallback_strafe_speed", 0.32);
    fallback_forward_speed_ = declare_parameter<double>("fallback_forward_speed", 0.18);
    fallback_yaw_rate_ = declare_parameter<double>("fallback_yaw_rate", 0.14);
    fallback_front_margin_ = declare_parameter<double>("fallback_front_margin", 0.30);
    fallback_side_margin_ = declare_parameter<double>("fallback_side_margin", 0.12);

    // weights (lower cost is better)
    w_goal_heading_ = declare_parameter<double>("w_goal_heading", 1.0);
    w_progress_     = declare_parameter<double>("w_progress", 2.5);
    w_clearance_    = declare_parameter<double>("w_clearance", 1.2);
    w_speed_        = declare_parameter<double>("w_speed", 0.2);
    w_smooth_       = declare_parameter<double>("w_smooth", 0.3);
    w_yaw_rate_     = declare_parameter<double>("w_yaw_rate", 0.8);

    // Extra recovery stage before fallback.
    full_search_if_empty_ = declare_parameter<bool>("full_search_if_empty", true);
    full_search_w_max_ = declare_parameter<double>("full_search_w_max", 0.25);
    full_search_collision_scale_ =
        declare_parameter<double>("full_search_collision_scale", 0.70);

    // Heading behavior: face the setpoint direction while moving.
    face_goal_to_setpoint_ = declare_parameter<bool>("face_goal_to_setpoint", true);
    face_goal_k_yaw_ = declare_parameter<double>("face_goal_k_yaw", 0.9);
    face_goal_deadband_deg_ = declare_parameter<double>("face_goal_deadband_deg", 3.0);
    face_goal_turn_only_deg_ = declare_parameter<double>("face_goal_turn_only_deg", 70.0);
    face_goal_mix_with_dwa_ = declare_parameter<double>("face_goal_mix_with_dwa", 0.80);
    face_goal_min_xy_scale_ = declare_parameter<double>("face_goal_min_xy_scale", 0.20);
    hold_yaw_near_goal_radius_ = declare_parameter<double>("hold_yaw_near_goal_radius", 0.30);

    // If true: publish vx,vy in world/map frame (recommended for PX4 offboard velocity in ENU).
    publish_world_cmd_ = declare_parameter<bool>("publish_world_cmd", true);
    publish_rollout_path_ = declare_parameter<bool>("publish_rollout_path", true);
    rollout_path_frame_ = declare_parameter<std::string>("rollout_path_frame", "base_link");
    publish_full_path_ = declare_parameter<bool>("publish_full_path", true);
    full_path_max_points_ = declare_parameter<int>("full_path_max_points", 3000);
    full_path_min_step_ = declare_parameter<double>("full_path_min_step", 0.05);

    // -------- ROS I/O --------
    auto qos_sensor = rclcpp::SensorDataQoS();
    auto qos_goal   = rclcpp::QoS(10).reliable();

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/local_position/odom", qos_sensor,
        [this](nav_msgs::msg::Odometry::SharedPtr msg){ odom_ = *msg; });

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_horizontal", qos_sensor,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ scan_ = *msg; });

    sub_goal_ = create_subscription<geometry_msgs::msg::Point>(
        "/drone_goal", qos_goal,
        [this](geometry_msgs::msg::Point::SharedPtr msg){ goal_ = *msg; });

    pub_cmd_ = create_publisher<geometry_msgs::msg::TwistStamped>("/planner_cmd_vel", 10);
    pub_rollout_path_ = create_publisher<nav_msgs::msg::Path>("/dwa/best_rollout_path", 10);
    pub_full_path_ = create_publisher<nav_msgs::msg::Path>("/dwa/full_path", 10);

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(control_dt_)),
        std::bind(&DwaLocalPlanner::loop, this));

    last_cmd_time_ = now();
    cmd_inited_ = false;
    RCLCPP_INFO(get_logger(), "DWA skeleton started.");
  }

private:
  struct Candidate {
    bool valid{false};
    double vx_b{0}, vy_b{0}, wz{0};
    double cost{std::numeric_limits<double>::infinity()};
    double min_clear{std::numeric_limits<double>::infinity()};
    double end_x{0}, end_y{0}, end_yaw{0};
  };

  struct ConeClearance {
    double dist{0.0};
    bool covered{false};
  };

  struct ClearanceForecast {
    double start_clear{0.0};
    double min_future_clear{0.0};
    double first_step_clear{0.0};
  };

  void loop() {
    if (!odom_ || !scan_ || !goal_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odom/scan/goal...");
      return;
    }

    const auto &odom = odom_.value();
    const auto &scan = scan_.value();
    const auto &goal = goal_.value();

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double z0 = odom.pose.pose.position.z;
    const double yaw0 = yaw_from_odom(odom);

    const double gx = goal.x;
    const double gy = goal.y;
    const double gz = goal.z;
    const double ex = gx - x0;
    const double ey = gy - y0;
    const double ez = gz - z0;
    const double d_goal = std::hypot(ex, ey);
    publish_full_path_from_odom(odom);

    // 1) Build obstacle points in WORLD frame from scan
    build_obstacles_world(scan, x0, y0, yaw0, obstacles_);

    // 2) Dynamic window around current command (or odom if you want)
    const rclcpp::Time t = now();
    const double dtw = clamp((t - last_cmd_time_).seconds(), 0.02, 0.20);

    const double vx_cur = cmd_inited_ ? vx_cmd_b_ : 0.0;
    const double vy_cur = cmd_inited_ ? vy_cmd_b_ : 0.0;
    const double wz_cur = cmd_inited_ ? wz_cmd_   : 0.0;

    const double vx_lo = clamp(vx_cur - ax_max_ * dtw, -v_max_,  v_max_);
    const double vx_hi = clamp(vx_cur + ax_max_ * dtw, -v_max_,  v_max_);
    const double vy_lo = clamp(vy_cur - ay_max_ * dtw, -vy_max_, vy_max_);
    const double vy_hi = clamp(vy_cur + ay_max_ * dtw, -vy_max_, vy_max_);
    const double wz_lo = clamp(wz_cur - aw_max_ * dtw, -w_max_,  w_max_);
    const double wz_hi = clamp(wz_cur + aw_max_ * dtw, -w_max_,  w_max_);

    // 3) Evaluate samples
    const double strict_collision_radius =
        std::max(collision_radius_, collision_from_safety_scale_ * safety_radius_);
    Candidate best = evaluate_window(vx_lo, vx_hi, vy_lo, vy_hi, wz_lo, wz_hi,
                                     x0, y0, yaw0, gx, gy, d_goal, strict_collision_radius);

    if (!best.valid) {
      // If strict envelope finds no candidate, retry with a smaller hard-collision radius.
      const double relaxed_collision_radius = std::max(0.18, 0.80 * strict_collision_radius);
      best = evaluate_window(vx_lo, vx_hi, vy_lo, vy_hi, wz_lo, wz_hi,
                             x0, y0, yaw0, gx, gy, d_goal, relaxed_collision_radius);
      if (best.valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1500,
            "DWA: strict search empty, using relaxed collision radius (strict=%.2f, relaxed=%.2f).",
            strict_collision_radius, relaxed_collision_radius);
      }
    }

    if (!best.valid && full_search_if_empty_) {
      // Last DWA attempt: widen search to full command envelope with limited yaw-rate.
      const double wide_collision_radius =
          std::max(0.16, full_search_collision_scale_ * strict_collision_radius);
      best = evaluate_window(
          -v_max_, v_max_, -vy_max_, vy_max_,
          -std::min(w_max_, full_search_w_max_), std::min(w_max_, full_search_w_max_),
          x0, y0, yaw0, gx, gy, d_goal, wide_collision_radius);
      if (best.valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1500,
            "DWA: recovered with full-envelope search (w_max=%.2f, collision=%.2f).",
            std::min(w_max_, full_search_w_max_), wide_collision_radius);
      }
    }

    // 4) Fallback if no valid plan: LiDAR-aware wall-follow sidestep
    if (!best.valid) {
      run_wall_follow_fallback(scan, x0, y0, yaw0, gx, gy, ez);
      return;
    }

    // 5) Publish chosen command (translation from DWA, yaw shaped to face setpoint)
    double vx_cmd = best.vx_b;
    double vy_cmd = best.vy_b;
    double wz_cmd = best.wz;
    const bool hold_yaw_near_goal = (d_goal <= hold_yaw_near_goal_radius_);

    if (face_goal_to_setpoint_ && !hold_yaw_near_goal && d_goal > 0.15) {
      const double goal_yaw = std::atan2(ey, ex);
      double yaw_err = wrap_pi(goal_yaw - yaw0);
      const double yaw_db = face_goal_deadband_deg_ * M_PI / 180.0;
      if (std::fabs(yaw_err) < yaw_db) yaw_err = 0.0;

      const double wz_goal = clamp(face_goal_k_yaw_ * yaw_err, -w_max_, w_max_);
      const double mix = clamp(face_goal_mix_with_dwa_, 0.0, 1.0);
      wz_cmd = (1.0 - mix) * best.wz + mix * wz_goal;

      const double turn_only = face_goal_turn_only_deg_ * M_PI / 180.0;
      if (std::fabs(yaw_err) > turn_only) {
        const double t =
            clamp((std::fabs(yaw_err) - turn_only) / std::max(1e-3, (M_PI - turn_only)), 0.0, 1.0);
        const double min_xy = clamp(face_goal_min_xy_scale_, 0.0, 1.0);
        const double scale = 1.0 - (1.0 - min_xy) * t;
        vx_cmd *= scale;
        vy_cmd *= scale;
      }
    }

    if (hold_yaw_near_goal) {
      wz_cmd = 0.0;
    }

    // Final anti-crash gate on the chosen command.
    // If blocked, switch to wall-follow fallback instead of just freezing in place.
    const ClearanceForecast pred =
        predict_clearance_forecast(vx_cmd, vy_cmd, wz_cmd, x0, y0, yaw0);
    const double hard_gate = safety_radius_ + final_cmd_clearance_margin_;
    const bool escaping_now =
        pred.first_step_clear > (pred.start_clear + std::max(0.0, final_cmd_escape_trend_eps_));
    const double relax_gate =
        std::max(collision_radius_ + 0.05, hard_gate - std::max(0.0, final_cmd_escape_relax_));
    const double gate_to_use = escaping_now ? relax_gate : hard_gate;
    if (pred.min_future_clear < gate_to_use) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "DWA cmd blocked by final safety gate: start=%.2f first=%.2f min_future=%.2f gate=%.2f "
          "(hard=%.2f relax=%.2f escaping=%d) -> wall-follow fallback",
          pred.start_clear, pred.first_step_clear, pred.min_future_clear, gate_to_use, hard_gate,
          relax_gate, static_cast<int>(escaping_now));
      run_wall_follow_fallback(scan, x0, y0, yaw0, gx, gy, ez);
      return;
    }

    const double vz_cmd = clamp(0.8 * ez, -vz_max_, vz_max_);
    publish_cmd(vx_cmd, vy_cmd, vz_cmd, wz_cmd);
    publish_rollout_path(x0, y0, yaw0, vx_cmd, vy_cmd, wz_cmd);
  }

  void build_obstacles_world(const sensor_msgs::msg::LaserScan &scan,
                             double px, double py, double yaw,
                             std::vector<ObPoint> &out) const {
    out.clear();
    out.reserve(scan.ranges.size() / std::max(1, scan_stride_) + 1);

    double ang = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i, ang += scan.angle_increment) {
      if (static_cast<int>(i) % std::max(1, scan_stride_) != 0) continue;

      const float rf = scan.ranges[i];
      if (!std::isfinite(rf)) continue;
      double r = clamp(static_cast<double>(rf), scan.range_min, std::min<double>(scan.range_max, max_use_range_));
      if (r <= scan.range_min + 1e-2) continue;
      if (r > obs_range_) continue;

      // ray angle is in sensor frame; assume scan is in base_link aligned with yaw
      const double aw = yaw + ang;
      out.push_back({px + r * std::cos(aw), py + r * std::sin(aw)});
    }
  }

  Candidate evaluate_window(double vx_lo, double vx_hi,
                            double vy_lo, double vy_hi,
                            double wz_lo, double wz_hi,
                            double x0, double y0, double yaw0,
                            double gx, double gy, double d_goal0,
                            double collision_radius) const {
    Candidate best;

    // Seed with an explicit hover candidate so planner always has a conservative fallback
    // when dynamic-window samples all get rejected.
    Candidate hover = rollout_and_score(0.0, 0.0, 0.0, x0, y0, yaw0, gx, gy, d_goal0, collision_radius);
    if (hover.valid) {
      best = hover;
    }

    const int nx = std::max(2, nx_);
    const int ny = std::max(2, ny_);
    const int nw = std::max(3, nw_);

    for (int ix = 0; ix < nx; ++ix) {
      const double fx = double(ix) / double(nx - 1);
      const double vx = vx_lo + fx * (vx_hi - vx_lo);

      for (int iy = 0; iy < ny; ++iy) {
        const double fy = double(iy) / double(ny - 1);
        const double vy = vy_lo + fy * (vy_hi - vy_lo);

        // cap body speed
        if (std::hypot(vx, vy) > v_max_) continue;

        for (int iw = 0; iw < nw; ++iw) {
          const double fw = double(iw) / double(nw - 1);
          const double wz = wz_lo + fw * (wz_hi - wz_lo);

          Candidate c = rollout_and_score(vx, vy, wz, x0, y0, yaw0, gx, gy, d_goal0, collision_radius);
          if (!c.valid) continue;
          if (c.cost < best.cost) best = c;
        }
      }
    }
    return best;
  }

  Candidate rollout_and_score(double vx_b, double vy_b, double wz,
                              double x0, double y0, double yaw0,
                              double gx, double gy, double d_goal0,
                              double collision_radius) const {
    Candidate c;
    c.vx_b = vx_b; c.vy_b = vy_b; c.wz = wz;

    double x = x0, y = y0, yaw = yaw0;
    double min_clear = std::numeric_limits<double>::infinity();
    const double safe2 = collision_radius * collision_radius;

    const int steps = std::max(1, int(std::ceil(horizon_ / sim_dt_)));

    // integrate forward
    for (int k = 0; k < steps; ++k) {
      yaw = wrap_pi(yaw + wz * sim_dt_);

      double vx_w, vy_w;
      rot_body_to_world(yaw, vx_b, vy_b, vx_w, vy_w);

      x += vx_w * sim_dt_;
      y += vy_w * sim_dt_;

      // collision check vs obstacle points
      for (const auto &ob : obstacles_) {
        const double dx = x - ob.x;
        const double dy = y - ob.y;
        const double d2 = dx*dx + dy*dy;
        if (d2 <= safe2) {
          c.valid = false;
          return c;
        }
        min_clear = std::min(min_clear, std::sqrt(d2));
      }
    }

    c.end_x = x; c.end_y = y; c.end_yaw = yaw;
    c.min_clear = std::isfinite(min_clear) ? min_clear : max_use_range_;

    // Hard safety validity: reject any candidate whose predicted clearance is below safety envelope.
    const double hard_clear = safety_radius_ + hard_clearance_margin_;
    if (c.min_clear < hard_clear) {
      c.valid = false;
      return c;
    }

    // ---- Costs (normalized-ish) ----
    // 1) Goal alignment: prefer displacement direction aligned with goal ray.
    const double gdx0 = gx - x0;
    const double gdy0 = gy - y0;
    const double gnorm = std::hypot(gdx0, gdy0);
    const double mdx = x - x0;
    const double mdy = y - y0;
    const double mnorm = std::hypot(mdx, mdy);
    const double align =
        (mdx * gdx0 + mdy * gdy0) / (std::max(1e-3, gnorm) * std::max(1e-3, mnorm));
    const double cost_goal_heading = clamp((1.0 - align) * 0.5, 0.0, 1.0);

    // 2) Progress: reduce distance-to-goal (must not go backwards too much)
    const double d1 = std::hypot(gx - x, gy - y);
    const double progress = (d_goal0 - d1);
    // Soft-penalize backward progress instead of hard reject to avoid empty-search lockups.
    const double cost_progress = clamp(1.0 - (progress / std::max(0.5, v_max_ * horizon_)), 0.0, 3.0);

    // 3) Clearance: penalize being close
    // knee near safety radius; fades out by ~ (safety + 2m)
    const double knee = safety_radius_ + 0.20;
    const double fade = knee + 2.0;
    double cost_clear = 0.0;
    if (c.min_clear <= knee) cost_clear = 1.0;
    else cost_clear = clamp((fade - c.min_clear) / std::max(1e-3, fade - knee), 0.0, 1.0);

    // 4) Speed: prefer higher body speed
    const double sp = std::hypot(vx_b, vy_b);
    const double cost_speed = 1.0 - clamp(sp / std::max(1e-3, v_max_), 0.0, 1.0);

    // 5) Smoothness: penalize switching from last cmd
    const double dv = std::hypot(vx_b - vx_cmd_b_, vy_b - vy_cmd_b_) / std::max(1e-3, v_max_);
    const double dw = std::fabs(wz - wz_cmd_) / std::max(1e-3, w_max_);
    const double cost_smooth = clamp(0.7 * dv + 0.3 * dw, 0.0, 1.5);

    // 6) Explicit anti-spin: keep yaw rate low unless truly needed.
    const double cost_yaw_rate = std::fabs(wz) / std::max(1e-3, w_max_);

    c.cost =
        w_goal_heading_ * cost_goal_heading +
        w_progress_     * cost_progress +
        w_clearance_    * cost_clear +
        w_speed_        * cost_speed +
        w_smooth_       * cost_smooth +
        w_yaw_rate_     * cost_yaw_rate;

    if (progress < 0.0) {
      c.cost += 1.2 * std::fabs(progress);
    }

    c.valid = true;
    return c;
  }

  ClearanceForecast predict_clearance_forecast(double vx_b,
                                               double vy_b,
                                               double wz,
                                               double x0,
                                               double y0,
                                               double yaw0) const {
    ClearanceForecast out{};
    double x = x0;
    double y = y0;
    double yaw = yaw0;
    out.start_clear = nearest_obstacle_distance(x0, y0);
    out.first_step_clear = out.start_clear;
    out.min_future_clear = std::numeric_limits<double>::infinity();
    const int steps = std::max(1, static_cast<int>(std::ceil(horizon_ / sim_dt_)));

    for (int k = 0; k < steps; ++k) {
      yaw = wrap_pi(yaw + wz * sim_dt_);
      double vx_w = 0.0, vy_w = 0.0;
      rot_body_to_world(yaw, vx_b, vy_b, vx_w, vy_w);
      x += vx_w * sim_dt_;
      y += vy_w * sim_dt_;
      const double d = nearest_obstacle_distance(x, y);
      if (k == 0) out.first_step_clear = d;
      out.min_future_clear = std::min(out.min_future_clear, d);
    }

    if (!std::isfinite(out.start_clear)) out.start_clear = max_use_range_;
    if (!std::isfinite(out.first_step_clear)) out.first_step_clear = out.start_clear;
    if (!std::isfinite(out.min_future_clear)) out.min_future_clear = out.first_step_clear;
    return out;
  }

  double nearest_obstacle_distance(double x, double y) const {
    if (obstacles_.empty()) return max_use_range_;
    double min_d = std::numeric_limits<double>::infinity();
    for (const auto &ob : obstacles_) {
      const double dx = x - ob.x;
      const double dy = y - ob.y;
      min_d = std::min(min_d, std::sqrt(dx * dx + dy * dy));
    }
    return std::isfinite(min_d) ? min_d : max_use_range_;
  }

  ConeClearance cone_clearance_info(const sensor_msgs::msg::LaserScan &scan,
                                    double center_ang,
                                    double half_cone_deg) const {
    const double half = half_cone_deg * M_PI / 180.0;
    double ang = scan.angle_min;
    double d = max_use_range_;
    size_t count = 0;

    for (float rf : scan.ranges) {
      if (std::fabs(wrap_pi(ang - center_ang)) <= half) {
        count++;
        double r = std::isfinite(rf) ? static_cast<double>(rf) : max_use_range_;
        r = clamp(r, scan.range_min, std::min<double>(scan.range_max, max_use_range_));
        d = std::min(d, r);
      }
      ang += scan.angle_increment;
    }

    if (count == 0) {
      return {0.0, false};
    }
    return {d, true};
  }

  void run_wall_follow_fallback(const sensor_msgs::msg::LaserScan &scan,
                                double x0, double y0, double yaw0,
                                double gx, double gy, double ez) {
    const ConeClearance front = cone_clearance_info(scan, 0.0, 20.0);
    const ConeClearance left = cone_clearance_info(scan, M_PI_2, 30.0);
    const ConeClearance right = cone_clearance_info(scan, -M_PI_2, 30.0);

    const double ex = gx - x0;
    const double ey = gy - y0;
    const double c = std::cos(yaw0);
    const double s = std::sin(yaw0);
    const double goal_by = -s * ex + c * ey;

    const double left_score = left.covered ? left.dist : 0.0;
    const double right_score = right.covered ? right.dist : 0.0;
    int side_sign = (left_score >= right_score) ? 1 : -1;

    // If both sides look similar, bias sidestep toward the side where the goal lies in body frame.
    if (std::fabs(left_score - right_score) < 0.25) {
      side_sign = (goal_by >= 0.0) ? 1 : -1;
    }

    const double best_side = std::max(left_score, right_score);
    const bool side_ok = best_side > (safety_radius_ + fallback_side_margin_);

    double vx_b = 0.0;
    double vy_b = 0.0;
    double wz = 0.0;

    if (side_ok) {
      const double side_scale =
          clamp((best_side - (safety_radius_ + fallback_side_margin_)) / 0.9, 0.0, 1.0);
      vy_b = static_cast<double>(side_sign) * fallback_strafe_speed_ * (0.35 + 0.65 * side_scale);

      const double front_dist = front.covered ? front.dist : 0.0;
      const double front_scale =
          clamp((front_dist - (safety_radius_ + fallback_front_margin_)) / 1.2, 0.0, 1.0);
      vx_b = fallback_forward_speed_ * front_scale;
    } else {
      // Boxed in: avoid blind backward motion; only small yaw to re-open free side.
      if (left.covered || right.covered) {
        wz = static_cast<double>(side_sign) * fallback_yaw_rate_;
      }
    }

    const ClearanceForecast pred = predict_clearance_forecast(vx_b, vy_b, wz, x0, y0, yaw0);
    const double hard_gate = safety_radius_ + final_cmd_clearance_margin_;
    const bool escaping_now =
        pred.first_step_clear > (pred.start_clear + std::max(0.0, final_cmd_escape_trend_eps_));
    const double relax_gate =
        std::max(collision_radius_ + 0.05, hard_gate - std::max(0.0, final_cmd_escape_relax_));
    const double gate_to_use = escaping_now ? relax_gate : hard_gate;
    if (pred.min_future_clear < gate_to_use) {
      if (side_ok && escaping_now) {
        vx_b = 0.0;
        vy_b = static_cast<double>(side_sign) * 0.20 * fallback_strafe_speed_;
        wz = 0.0;
      } else {
        vx_b = 0.0;
        vy_b = 0.0;
        wz = clamp(wz, -0.30 * w_max_, 0.30 * w_max_);
      }
    }

    const double d_goal = std::hypot(gx - x0, gy - y0);
    if (d_goal <= hold_yaw_near_goal_radius_) {
      wz = 0.0;
    }

    const double vz = clamp(0.8 * ez, -vz_max_, vz_max_);
    publish_cmd(vx_b, vy_b, vz, wz);
    publish_rollout_path(x0, y0, yaw0, vx_b, vy_b, wz);

    const double min_clear_now = nearest_obstacle_distance(x0, y0);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No valid DWA candidate -> wall-follow fallback. side=%s front=%.2f left=%.2f right=%.2f min_clear=%.2f",
        (side_sign > 0) ? "left" : "right", front.dist, left.dist, right.dist, min_clear_now);
  }

  void publish_cmd(double vx_b, double vy_b, double vz, double wz) {
    // store cmd state (used for window + smoothing)
    vx_cmd_b_ = vx_b;
    vy_cmd_b_ = vy_b;
    wz_cmd_   = wz;
    cmd_inited_ = true;
    last_cmd_time_ = now();

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = last_cmd_time_;

    // Publish in WORLD/ENU (recommended for MAVROS local velocity setpoints),
    // but planned variables are in body frame, so rotate by current yaw from odom.
    double vx_out = vx_b, vy_out = vy_b;
    if (publish_world_cmd_ && odom_) {
      const double yaw = yaw_from_odom(odom_.value());
      rot_body_to_world(yaw, vx_b, vy_b, vx_out, vy_out);
      msg.header.frame_id = "map";
    } else {
      msg.header.frame_id = "base_link";
    }

    msg.twist.linear.x  = clamp(vx_out, -v_max_, v_max_);
    msg.twist.linear.y  = clamp(vy_out, -v_max_, v_max_);
    msg.twist.linear.z  = clamp(vz, -vz_max_, vz_max_);
    msg.twist.angular.z = clamp(wz, -w_max_, w_max_);

    pub_cmd_->publish(msg);
  }

  void publish_rollout_path(double x0, double y0, double yaw0,
                            double vx_b, double vy_b, double wz) {
    if (!publish_rollout_path_ || !pub_rollout_path_) return;

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    const bool use_base_link_frame =
        (rollout_path_frame_ == "base_link") || (rollout_path_frame_ == "link");
    path.header.frame_id = use_base_link_frame ? "base_link" : "map";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = use_base_link_frame ? 0.0 : x0;
    pose.pose.position.y = use_base_link_frame ? 0.0 : y0;
    pose.pose.position.z = 0.0;
    path.poses.push_back(pose);

    const int steps = std::max(1, static_cast<int>(std::ceil(horizon_ / sim_dt_)));
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double yaw = yaw0;
    for (int k = 0; k < steps; ++k) {
      if (use_base_link_frame) {
        x += vx_b * sim_dt_;
        y += vy_b * sim_dt_;
      } else {
        yaw = wrap_pi(yaw + wz * sim_dt_);
        double vx_w = 0.0, vy_w = 0.0;
        rot_body_to_world(yaw, vx_b, vy_b, vx_w, vy_w);
        x += vx_w * sim_dt_;
        y += vy_w * sim_dt_;
      }
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      path.poses.push_back(pose);
    }

    pub_rollout_path_->publish(path);
  }

  void publish_full_path_from_odom(const nav_msgs::msg::Odometry &odom) {
    if (!publish_full_path_ || !pub_full_path_) return;

    const std::string frame_id = odom.header.frame_id.empty() ? "map" : odom.header.frame_id;
    const rclcpp::Time stamp(odom.header.stamp);

    if (full_path_msg_.header.frame_id != frame_id ||
        (!full_path_msg_.poses.empty() && stamp < rclcpp::Time(full_path_msg_.header.stamp))) {
      full_path_msg_.poses.clear();
      have_last_full_path_pose_ = false;
    }

    full_path_msg_.header.frame_id = frame_id;

    const auto &p = odom.pose.pose.position;
    bool should_append = !have_last_full_path_pose_;
    if (!should_append) {
      const double dx = p.x - last_full_path_x_;
      const double dy = p.y - last_full_path_y_;
      const double dz = p.z - last_full_path_z_;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      should_append = dist >= std::max(0.01, full_path_min_step_);
    }

    if (should_append) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = odom.header.stamp;
      pose.header.frame_id = frame_id;
      pose.pose = odom.pose.pose;
      full_path_msg_.poses.push_back(pose);

      last_full_path_x_ = p.x;
      last_full_path_y_ = p.y;
      last_full_path_z_ = p.z;
      have_last_full_path_pose_ = true;

      const int max_points = std::max(100, full_path_max_points_);
      if (static_cast<int>(full_path_msg_.poses.size()) > max_points) {
        const size_t trim = full_path_msg_.poses.size() - static_cast<size_t>(max_points);
        full_path_msg_.poses.erase(full_path_msg_.poses.begin(),
                                   full_path_msg_.poses.begin() + trim);
      }
    }

    full_path_msg_.header.stamp = odom.header.stamp;
    pub_full_path_->publish(full_path_msg_);
  }

private:
  // params
  double v_max_{0.75}, vy_max_{0.55}, w_max_{0.45}, vz_max_{1.0};
  double ax_max_{1.0}, ay_max_{1.0}, aw_max_{1.2};
  double control_dt_{0.05}, sim_dt_{0.10}, horizon_{1.6};
  int nx_{9}, ny_{7}, nw_{11};
  double max_use_range_{10.0}, safety_radius_{0.80}, collision_radius_{0.32}, obs_range_{8.0};
  double collision_from_safety_scale_{0.85};
  double hard_clearance_margin_{0.05};
  double final_cmd_clearance_margin_{0.08};
  double final_cmd_escape_relax_{0.20};
  double final_cmd_escape_trend_eps_{0.03};
  double fallback_strafe_speed_{0.32}, fallback_forward_speed_{0.18}, fallback_yaw_rate_{0.14};
  double fallback_front_margin_{0.30}, fallback_side_margin_{0.12};
  int scan_stride_{2};
  double w_goal_heading_{1.0}, w_progress_{2.5}, w_clearance_{1.2}, w_speed_{0.2}, w_smooth_{0.3};
  double w_yaw_rate_{0.8};
  bool full_search_if_empty_{true};
  double full_search_w_max_{0.25};
  double full_search_collision_scale_{0.70};
  bool face_goal_to_setpoint_{true};
  double face_goal_k_yaw_{0.9};
  double face_goal_deadband_deg_{3.0};
  double face_goal_turn_only_deg_{70.0};
  double face_goal_mix_with_dwa_{0.80};
  double face_goal_min_xy_scale_{0.20};
  double hold_yaw_near_goal_radius_{0.30};
  bool publish_world_cmd_{true};
  bool publish_rollout_path_{true};
  std::string rollout_path_frame_{"base_link"};
  bool publish_full_path_{true};
  int full_path_max_points_{3000};
  double full_path_min_step_{0.05};

  // state
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  std::optional<geometry_msgs::msg::Point> goal_;
  nav_msgs::msg::Path full_path_msg_;
  bool have_last_full_path_pose_{false};
  double last_full_path_x_{0.0};
  double last_full_path_y_{0.0};
  double last_full_path_z_{0.0};

  std::vector<ObPoint> obstacles_;

  bool cmd_inited_{false};
  double vx_cmd_b_{0.0}, vy_cmd_b_{0.0}, wz_cmd_{0.0};
  rclcpp::Time last_cmd_time_;

  // ros
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_rollout_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_full_path_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaLocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
