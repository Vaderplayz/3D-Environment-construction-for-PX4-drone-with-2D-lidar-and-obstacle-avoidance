#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace {
inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

inline double yaw_from_odom(const nav_msgs::msg::Odometry &odom) {
  const auto &q = odom.pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class MissionInterceptorNode : public rclcpp::Node {
 public:
  MissionInterceptorNode() : Node("mission_interceptor") {
    declare_params();

    auto qos_sensor = rclcpp::SensorDataQoS();
    auto qos_state = rclcpp::QoS(10).reliable();
    auto qos_cmd = rclcpp::QoS(10).reliable();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        state_topic_, qos_state,
        [this](mavros_msgs::msg::State::SharedPtr msg) {
          mavros_state_ = *msg;
          have_state_ = true;
          last_state_time_ = now();
        });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, qos_sensor,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          scan_ = *msg;
          have_scan_ = true;
          last_scan_time_ = now();
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, qos_sensor,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_ = *msg;
          have_odom_ = true;
          last_odom_time_ = now();
        });

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic_, qos_cmd);
    override_pub_ = create_publisher<std_msgs::msg::Bool>(override_topic_, qos_cmd);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_out_topic_, qos_cmd);
    status_pub_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, qos_cmd);

    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    const auto control_period =
        std::chrono::duration<double>(1.0 / std::max(1.0, setpoint_hz_));
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(control_period),
        std::bind(&MissionInterceptorNode::control_loop, this));

    status_timer_ =
        create_wall_timer(500ms, std::bind(&MissionInterceptorNode::publish_status, this));

    state_enter_time_ = now();
    next_mode_request_time_ = now();
    publish_state_name();
    publish_override(false);

    RCLCPP_INFO(get_logger(),
                "mission_interceptor started. scan='%s' odom='%s' state='%s' cmd='%s'",
                scan_topic_.c_str(), odom_topic_.c_str(), state_topic_.c_str(),
                cmd_topic_.c_str());
  }

 private:
  enum class State {
    kMonitor,
    kHazardConfirm,
    kOffboardWarmup,
    kOffboardRequest,
    kBrake,
    kSidestep,
    kForwardClear,
    kClearVerify,
    kMissionResumeRequest,
    kResumeGuard,
    kCooldown,
    kLoiterLatch,
  };

  struct Cmd {
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
    double wz{0.0};
    bool publish{false};
  };

  void declare_params() {
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan_horizontal");
    odom_topic_ =
        declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");

    cmd_topic_ =
        declare_parameter<std::string>("cmd_topic", "/mission_interceptor/cmd_vel");
    override_topic_ = declare_parameter<std::string>("override_topic",
                                                     "/mission_interceptor/override_active");
    state_out_topic_ =
        declare_parameter<std::string>("state_out_topic", "/mission_obs_avoid/state");
    status_topic_ =
        declare_parameter<std::string>("status_topic", "/mission_obs_avoid/status");

    cmd_frame_id_ = declare_parameter<std::string>("cmd_frame_id", "map");

    hazard_distance_ = declare_parameter<double>("hazard_distance", 2.5);
    emergency_distance_ = declare_parameter<double>("emergency_distance", 1.0);
    clear_distance_ = declare_parameter<double>("clear_distance", 3.0);
    front_fov_deg_ = declare_parameter<double>("front_fov_deg", 40.0);

    hazard_confirm_sec_ = declare_parameter<double>("hazard_confirm_sec", 0.30);
    offboard_warmup_sec_ = declare_parameter<double>("offboard_warmup_sec", 1.00);
    setpoint_hz_ = declare_parameter<double>("setpoint_hz", 20.0);

    set_mode_retry_sec_ = declare_parameter<double>("set_mode_retry_sec", 1.0);
    max_set_mode_retries_ = declare_parameter<int>("max_set_mode_retries", 5);

    brake_sec_ = declare_parameter<double>("brake_sec", 0.8);
    sidestep_speed_ = declare_parameter<double>("sidestep_speed", 0.6);
    sidestep_min_dist_ = declare_parameter<double>("sidestep_min_dist", 1.5);
    sidestep_max_sec_ = declare_parameter<double>("sidestep_max_sec", 4.0);
    forward_speed_ = declare_parameter<double>("forward_speed", 0.8);
    forward_clear_dist_ = declare_parameter<double>("forward_clear_dist", 2.0);

    clear_hold_sec_ = declare_parameter<double>("clear_hold_sec", 1.5);
    max_intercept_sec_ = declare_parameter<double>("max_intercept_sec", 15.0);
    resume_grace_sec_ = declare_parameter<double>("resume_grace_sec", 2.5);
    cooldown_sec_ = declare_parameter<double>("cooldown_sec", 5.0);
    latch_clear_hold_sec_ = declare_parameter<double>("latch_clear_hold_sec", 3.0);
    sensor_timeout_sec_ = declare_parameter<double>("sensor_timeout_sec", 0.6);

    side_clear_percentile_ = declare_parameter<double>("side_clear_percentile", 0.50);
    side_clear_tie_margin_ = declare_parameter<double>("side_clear_tie_margin", 0.10);
  }

  void control_loop() {
    const auto tnow = now();

    const bool sensors_fresh = have_all_sensors_fresh(tnow);
    const double front_min = sensors_fresh ? compute_front_min(scan_) : std::numeric_limits<double>::infinity();
    const bool hazard = std::isfinite(front_min) && (front_min < hazard_distance_);
    const bool emergency = std::isfinite(front_min) && (front_min < emergency_distance_);
    const bool clear = std::isfinite(front_min) && (front_min >= clear_distance_);

    last_front_min_ = front_min;
    last_hazard_ = hazard;
    last_emergency_ = emergency;

    if (state_requires_fresh_sensors(state_) && !sensors_fresh) {
      enter_loiter_latch("sensor timeout during active state");
    }

    Cmd cmd;
    bool override = false;

    switch (state_) {
      case State::kMonitor: {
        if (!sensors_fresh) {
          break;
        }
        if (!mavros_state_.armed) {
          break;
        }
        if (is_mode("AUTO.MISSION") && hazard) {
          hazard_confirm_start_ = tnow;
          transition(State::kHazardConfirm, "hazard detected in AUTO.MISSION");
        }
        break;
      }

      case State::kHazardConfirm: {
        if (!sensors_fresh) {
          break;
        }
        if (!is_mode("AUTO.MISSION") || !hazard) {
          transition(State::kMonitor, "hazard cleared or left mission mode");
          break;
        }

        if (!hazard_confirm_start_.has_value()) {
          hazard_confirm_start_ = tnow;
        }
        if (emergency || elapsed_from(*hazard_confirm_start_, tnow) >= hazard_confirm_sec_) {
          intercept_start_time_ = tnow;
          transition(State::kOffboardWarmup, emergency ? "emergency threshold" : "hazard confirmed");
        }
        break;
      }

      case State::kOffboardWarmup: {
        override = true;
        cmd = zero_cmd();
        cmd.publish = true;

        if (intercept_timed_out(tnow)) {
          enter_loiter_latch("intercept timeout in warmup");
          break;
        }

        if (elapsed_in_state(tnow) >= offboard_warmup_sec_) {
          transition(State::kOffboardRequest, "warmup done");
        }
        break;
      }

      case State::kOffboardRequest: {
        override = true;
        cmd = zero_cmd();
        cmd.publish = true;

        if (intercept_timed_out(tnow)) {
          enter_loiter_latch("intercept timeout waiting OFFBOARD");
          break;
        }

        if (is_mode("OFFBOARD")) {
          transition(State::kBrake, "OFFBOARD active");
          break;
        }

        ensure_mode_with_retry("OFFBOARD", tnow,
                               "OFFBOARD mode request failed too many times");
        break;
      }

      case State::kBrake: {
        override = true;
        cmd = zero_cmd();
        cmd.publish = true;

        if (intercept_timed_out(tnow)) {
          enter_loiter_latch("intercept timeout in brake");
          break;
        }

        if (elapsed_in_state(tnow) >= brake_sec_) {
          select_sidestep_direction();
          capture_phase_reference(PhaseRef::kSidestep);
          transition(State::kSidestep, "brake complete");
        }
        break;
      }

      case State::kSidestep: {
        override = true;
        cmd.publish = true;
        cmd.vy = sidestep_sign_ * sidestep_speed_;

        if (intercept_timed_out(tnow)) {
          enter_loiter_latch("intercept timeout in sidestep");
          break;
        }

        const double progress = sidestep_progress();
        if (progress >= sidestep_min_dist_ || elapsed_in_state(tnow) >= sidestep_max_sec_) {
          capture_phase_reference(PhaseRef::kForward);
          transition(State::kForwardClear, "sidestep complete");
        }
        break;
      }

      case State::kForwardClear: {
        override = true;
        cmd.publish = true;
        cmd.vx = forward_speed_;

        if (intercept_timed_out(tnow)) {
          enter_loiter_latch("intercept timeout in forward clear");
          break;
        }

        if (forward_progress() >= forward_clear_dist_) {
          clear_hold_start_.reset();
          transition(State::kClearVerify, "forward distance reached");
        }
        break;
      }

      case State::kClearVerify: {
        override = true;
        cmd = zero_cmd();
        cmd.publish = true;

        if (intercept_timed_out(tnow)) {
          enter_loiter_latch("intercept timeout in clear verify");
          break;
        }

        if (clear) {
          if (!clear_hold_start_.has_value()) {
            clear_hold_start_ = tnow;
          }
          if (elapsed_from(*clear_hold_start_, tnow) >= clear_hold_sec_) {
            transition(State::kMissionResumeRequest, "clear verified");
          }
        } else {
          clear_hold_start_.reset();
        }
        break;
      }

      case State::kMissionResumeRequest: {
        override = true;
        cmd = zero_cmd();
        cmd.publish = true;

        if (intercept_timed_out(tnow)) {
          enter_loiter_latch("intercept timeout during mission resume request");
          break;
        }

        if (is_mode("AUTO.MISSION")) {
          transition(State::kResumeGuard, "AUTO.MISSION restored");
          break;
        }

        ensure_mode_with_retry("AUTO.MISSION", tnow,
                               "AUTO.MISSION request failed too many times");
        break;
      }

      case State::kResumeGuard: {
        if (!sensors_fresh) {
          enter_loiter_latch("sensor timeout in resume guard");
          break;
        }

        if (is_mode("AUTO.MISSION") && hazard) {
          enter_loiter_latch("hazard retriggered during resume guard");
          break;
        }

        if (elapsed_in_state(tnow) >= resume_grace_sec_) {
          transition(State::kCooldown, "resume guard passed");
        }
        break;
      }

      case State::kCooldown: {
        if (elapsed_in_state(tnow) >= cooldown_sec_) {
          transition(State::kMonitor, "cooldown complete");
        }
        break;
      }

      case State::kLoiterLatch: {
        if (!is_mode("AUTO.LOITER")) {
          ensure_mode_with_retry("AUTO.LOITER", tnow,
                                 "AUTO.LOITER request failed too many times");
        }

        if (sensors_fresh && is_mode("AUTO.MISSION") && !hazard) {
          if (!latch_clear_start_.has_value()) {
            latch_clear_start_ = tnow;
          }
          if (elapsed_from(*latch_clear_start_, tnow) >= latch_clear_hold_sec_) {
            transition(State::kMonitor,
                       "operator resumed mission and hazard stayed clear");
          }
        } else {
          latch_clear_start_.reset();
        }
        break;
      }
    }

    publish_override(override);
    if (override && cmd.publish) {
      publish_cmd(cmd.vx, cmd.vy, cmd.vz, cmd.wz);
    }

    publish_state_name();
  }

  void publish_status() {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "mission_obs_avoid";
    st.hardware_id = "px4_mavros";
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = state_name(state_);

    st.values.push_back(kv("state", state_name(state_)));
    st.values.push_back(kv("mode", have_state_ ? mavros_state_.mode : "<none>"));
    st.values.push_back(kv("armed", have_state_ && mavros_state_.armed ? "true" : "false"));
    st.values.push_back(kv("override", last_override_ ? "true" : "false"));
    st.values.push_back(kv("hazard", last_hazard_ ? "true" : "false"));
    st.values.push_back(kv("emergency", last_emergency_ ? "true" : "false"));
    st.values.push_back(kv("front_min_m", to_fixed(last_front_min_)));
    st.values.push_back(kv("sidestep_sign", std::to_string(sidestep_sign_)));
    st.values.push_back(kv("mode_retries", std::to_string(state_mode_retries_)));

    if (state_ == State::kLoiterLatch) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "LOITER_LATCH";
    }

    arr.status.push_back(st);
    status_pub_->publish(arr);
  }

  void publish_state_name() {
    const std::string name = state_name(state_);
    if (name == last_state_name_published_) {
      return;
    }
    last_state_name_published_ = name;
    std_msgs::msg::String msg;
    msg.data = name;
    state_pub_->publish(msg);
  }

  void publish_override(bool active) {
    last_override_ = active;
    std_msgs::msg::Bool msg;
    msg.data = active;
    override_pub_->publish(msg);
  }

  void publish_cmd(double vx, double vy, double vz, double wz) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = cmd_frame_id_;
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.linear.z = vz;
    cmd.twist.angular.z = wz;
    cmd_pub_->publish(cmd);
  }

  Cmd zero_cmd() const {
    return Cmd{};
  }

  bool have_all_sensors_fresh(const rclcpp::Time &tnow) const {
    if (!have_state_ || !have_scan_ || !have_odom_) {
      return false;
    }
    return is_fresh(last_state_time_, tnow) && is_fresh(last_scan_time_, tnow) &&
           is_fresh(last_odom_time_, tnow);
  }

  bool state_requires_fresh_sensors(State s) const {
    return s == State::kHazardConfirm || s == State::kOffboardWarmup ||
           s == State::kOffboardRequest || s == State::kBrake || s == State::kSidestep ||
           s == State::kForwardClear || s == State::kClearVerify ||
           s == State::kMissionResumeRequest || s == State::kResumeGuard;
  }

  bool is_fresh(const rclcpp::Time &stamp, const rclcpp::Time &tnow) const {
    return (tnow - stamp).seconds() <= sensor_timeout_sec_;
  }

  bool is_mode(const std::string &target) const {
    return have_state_ && mavros_state_.mode == target;
  }

  void ensure_mode_with_retry(const std::string &target, const rclcpp::Time &tnow,
                              const std::string &failure_reason) {
    if (is_mode(target)) {
      return;
    }

    if (mode_request_pending_) {
      return;
    }

    if (tnow < next_mode_request_time_) {
      return;
    }

    if (state_mode_retries_ >= max_set_mode_retries_) {
      enter_loiter_latch(failure_reason);
      return;
    }

    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for /mavros/set_mode service...");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = target;

    mode_request_pending_ = true;
    mode_request_target_ = target;
    state_mode_retries_++;
    next_mode_request_time_ = tnow + rclcpp::Duration::from_seconds(set_mode_retry_sec_);

    auto cb = [this, target](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      mode_request_pending_ = false;
      try {
        const auto resp = future.get();
        if (resp && resp->mode_sent) {
          RCLCPP_INFO(get_logger(), "SetMode request accepted for '%s'", target.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "SetMode request rejected for '%s'", target.c_str());
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "SetMode '%s' request exception: %s", target.c_str(),
                    e.what());
      }
    };

    set_mode_client_->async_send_request(req, cb);
  }

  void enter_loiter_latch(const std::string &reason) {
    if (state_ != State::kLoiterLatch) {
      RCLCPP_WARN(get_logger(), "Entering LOITER_LATCH: %s", reason.c_str());
    }
    transition(State::kLoiterLatch, reason);
  }

  void transition(State next, const std::string &reason) {
    if (state_ == next) {
      return;
    }

    const auto from = state_name(state_);
    const auto to = state_name(next);
    state_ = next;
    state_enter_time_ = now();
    clear_hold_start_.reset();
    mode_request_pending_ = false;
    mode_request_target_.clear();
    state_mode_retries_ = 0;
    next_mode_request_time_ = state_enter_time_;

    if (next == State::kOffboardWarmup) {
      intercept_start_time_ = state_enter_time_;
    }

    if (next == State::kLoiterLatch) {
      latch_clear_start_.reset();
    }

    RCLCPP_INFO(get_logger(), "[FSM] %s -> %s (%s)", from.c_str(), to.c_str(),
                reason.c_str());
    publish_state_name();
  }

  bool intercept_timed_out(const rclcpp::Time &tnow) const {
    if (!intercept_start_time_.has_value()) {
      return false;
    }
    return elapsed_from(*intercept_start_time_, tnow) > max_intercept_sec_;
  }

  double elapsed_in_state(const rclcpp::Time &tnow) const {
    return elapsed_from(state_enter_time_, tnow);
  }

  static double elapsed_from(const rclcpp::Time &start, const rclcpp::Time &tnow) {
    return (tnow - start).seconds();
  }

  enum class PhaseRef {
    kSidestep,
    kForward,
  };

  void capture_phase_reference(PhaseRef phase) {
    if (!have_odom_) {
      return;
    }

    const auto &p = odom_.pose.pose.position;
    const double yaw = yaw_from_odom(odom_);

    if (phase == PhaseRef::kSidestep) {
      sidestep_ref_x_ = p.x;
      sidestep_ref_y_ = p.y;
      sidestep_ref_heading_ = yaw;
    } else {
      forward_ref_x_ = p.x;
      forward_ref_y_ = p.y;
      forward_ref_heading_ = yaw;
    }
  }

  double sidestep_progress() const {
    if (!have_odom_) {
      return 0.0;
    }
    const auto &p = odom_.pose.pose.position;
    const double dx = p.x - sidestep_ref_x_;
    const double dy = p.y - sidestep_ref_y_;
    const double lx = -std::sin(sidestep_ref_heading_);
    const double ly = std::cos(sidestep_ref_heading_);
    const double lateral = dx * lx + dy * ly;
    return sidestep_sign_ * lateral;
  }

  double forward_progress() const {
    if (!have_odom_) {
      return 0.0;
    }
    const auto &p = odom_.pose.pose.position;
    const double dx = p.x - forward_ref_x_;
    const double dy = p.y - forward_ref_y_;
    const double fx = std::cos(forward_ref_heading_);
    const double fy = std::sin(forward_ref_heading_);
    return dx * fx + dy * fy;
  }

  void select_sidestep_direction() {
    const double left_clear = sector_percentile(scan_, deg2rad(20.0), deg2rad(160.0),
                                                side_clear_percentile_);
    const double right_clear = sector_percentile(scan_, deg2rad(-160.0), deg2rad(-20.0),
                                                 side_clear_percentile_);

    if (left_clear > right_clear + side_clear_tie_margin_) {
      sidestep_sign_ = +1;
    } else if (right_clear > left_clear + side_clear_tie_margin_) {
      sidestep_sign_ = -1;
    } else {
      sidestep_sign_ = (last_sidestep_sign_ == 1) ? -1 : 1;
    }

    last_sidestep_sign_ = sidestep_sign_;
    RCLCPP_INFO(get_logger(),
                "Sidestep selection: sign=%d left_clear=%.2f right_clear=%.2f",
                sidestep_sign_, left_clear, right_clear);
  }

  double compute_front_min(const sensor_msgs::msg::LaserScan &scan) const {
    if (scan.ranges.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    const double half = deg2rad(front_fov_deg_ * 0.5);
    double best = std::numeric_limits<double>::infinity();

    double ang = scan.angle_min;
    for (const float rf : scan.ranges) {
      if (std::fabs(ang) <= half && std::isfinite(rf)) {
        const double r = clamp(static_cast<double>(rf), scan.range_min, scan.range_max);
        if (r > scan.range_min + 1e-3) {
          best = std::min(best, r);
        }
      }
      ang += scan.angle_increment;
    }

    return best;
  }

  double sector_percentile(const sensor_msgs::msg::LaserScan &scan, double lo, double hi,
                           double percentile) const {
    std::vector<double> vals;
    vals.reserve(scan.ranges.size() / 4);

    double ang = scan.angle_min;
    for (const float rf : scan.ranges) {
      if (ang >= lo && ang <= hi && std::isfinite(rf)) {
        const double r = clamp(static_cast<double>(rf), scan.range_min, scan.range_max);
        if (r > scan.range_min + 1e-3) {
          vals.push_back(r);
        }
      }
      ang += scan.angle_increment;
    }

    if (vals.empty()) {
      return 0.0;
    }

    std::sort(vals.begin(), vals.end());
    const double p = clamp(percentile, 0.0, 1.0);
    const size_t idx = static_cast<size_t>(std::lround(p * static_cast<double>(vals.size() - 1)));
    return vals[idx];
  }

  static double deg2rad(double deg) { return deg * M_PI / 180.0; }

  static diagnostic_msgs::msg::KeyValue kv(const std::string &k, const std::string &v) {
    diagnostic_msgs::msg::KeyValue x;
    x.key = k;
    x.value = v;
    return x;
  }

  static std::string to_fixed(double x) {
    if (!std::isfinite(x)) {
      return "inf";
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.3f", x);
    return std::string(buf);
  }

  static std::string state_name(State s) {
    switch (s) {
      case State::kMonitor:
        return "MONITOR";
      case State::kHazardConfirm:
        return "HAZARD_CONFIRM";
      case State::kOffboardWarmup:
        return "OFFBOARD_WARMUP";
      case State::kOffboardRequest:
        return "OFFBOARD_REQUEST";
      case State::kBrake:
        return "BRAKE";
      case State::kSidestep:
        return "SIDESTEP";
      case State::kForwardClear:
        return "FORWARD_CLEAR";
      case State::kClearVerify:
        return "CLEAR_VERIFY";
      case State::kMissionResumeRequest:
        return "MISSION_RESUME_REQUEST";
      case State::kResumeGuard:
        return "RESUME_GUARD";
      case State::kCooldown:
        return "COOLDOWN";
      case State::kLoiterLatch:
        return "LOITER_LATCH";
      default:
        return "UNKNOWN";
    }
  }

  std::string state_topic_;
  std::string scan_topic_;
  std::string odom_topic_;

  std::string cmd_topic_;
  std::string override_topic_;
  std::string state_out_topic_;
  std::string status_topic_;
  std::string cmd_frame_id_;

  double hazard_distance_{2.5};
  double emergency_distance_{1.0};
  double clear_distance_{3.0};
  double front_fov_deg_{40.0};

  double hazard_confirm_sec_{0.30};
  double offboard_warmup_sec_{1.0};
  double setpoint_hz_{20.0};

  double set_mode_retry_sec_{1.0};
  int max_set_mode_retries_{5};

  double brake_sec_{0.8};
  double sidestep_speed_{0.6};
  double sidestep_min_dist_{1.5};
  double sidestep_max_sec_{4.0};
  double forward_speed_{0.8};
  double forward_clear_dist_{2.0};

  double clear_hold_sec_{1.5};
  double max_intercept_sec_{15.0};
  double resume_grace_sec_{2.5};
  double cooldown_sec_{5.0};
  double latch_clear_hold_sec_{3.0};
  double sensor_timeout_sec_{0.6};

  double side_clear_percentile_{0.5};
  double side_clear_tie_margin_{0.1};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  mavros_msgs::msg::State mavros_state_;
  sensor_msgs::msg::LaserScan scan_;
  nav_msgs::msg::Odometry odom_;

  bool have_state_{false};
  bool have_scan_{false};
  bool have_odom_{false};

  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  State state_{State::kMonitor};
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  std::string last_state_name_published_;

  std::optional<rclcpp::Time> hazard_confirm_start_;
  std::optional<rclcpp::Time> intercept_start_time_;
  std::optional<rclcpp::Time> clear_hold_start_;
  std::optional<rclcpp::Time> latch_clear_start_;

  bool mode_request_pending_{false};
  std::string mode_request_target_;
  int state_mode_retries_{0};
  rclcpp::Time next_mode_request_time_{0, 0, RCL_ROS_TIME};

  int sidestep_sign_{1};
  int last_sidestep_sign_{-1};
  double sidestep_ref_x_{0.0};
  double sidestep_ref_y_{0.0};
  double sidestep_ref_heading_{0.0};
  double forward_ref_x_{0.0};
  double forward_ref_y_{0.0};
  double forward_ref_heading_{0.0};

  double last_front_min_{std::numeric_limits<double>::infinity()};
  bool last_hazard_{false};
  bool last_emergency_{false};
  bool last_override_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionInterceptorNode>());
  rclcpp::shutdown();
  return 0;
}
